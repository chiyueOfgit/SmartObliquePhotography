#include "pch.h"
#include "HoleRepairer.h"
#include "PointCloudRetouchInterface.h"
#include "PointCloudRetouchManager.h"
#include "PointCloudRetouchConfig.h"
#include "ObliquePhotographyDataInterface.h"
#include "PointCloudVisualizer.h"
#include "VisualizationInterface.h"

#include <fstream>
#include <boost/archive/text_oarchive.hpp> 
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>

//测试用例列表：
// LatticesProjectionBaseTest:通过不同空洞边界点正确生成对应栅格，并将边界点投影到正确栅格内；
//	* Lattices_Projection_BaseTest_1: 整个场景一个小空洞；
//  * Lattices_Projection_BaseTest_2: 场景有五个空洞；
//  * Lattices_Generation_Test:		  测试通过给定包围盒及平面能否正确均匀划分格子
//  * Points_Projection_Test:		  测试能否将点投影到对应格子内
//  * OriginInfos_Filling_Test:		  测试能否通过格子内的点生成原始颜色和高度

#define ENABLE_VISUALIZER false

#define Epsilon 1e-3

using namespace hiveObliquePhotography::PointCloudRetouch;

const auto RetouchConfigFile = TESTMODEL_DIR + std::string("Config/Test018_PointCloudRetouchConfig.xml");
const auto HoleRepairerConfigFile = TESTMODEL_DIR + std::string("Config/Test020_HoleRepairerConfig.xml"); 
const auto DataPath = TESTMODEL_DIR + std::string("Test018_Model/");
const std::vector<std::string> ModelNames{ "one_hole", "five_holes" };

class TestLatticesProjection : public testing::Test
{
protected:
	void SetUp() override
	{
		m_TestNumber++;

		m_pRetouchConfig = new CPointCloudRetouchConfig;
		ASSERT_EQ(hiveConfig::hiveParseConfig(RetouchConfigFile, hiveConfig::EConfigType::XML, m_pRetouchConfig), hiveConfig::EParseResult::SUCCEED);
		m_pCloud.reset(new PointCloud_t);
		m_pCloud = hiveObliquePhotography::hiveInitPointCloudScene({ DataPath + ModelNames[m_TestNumber] + ".ply" });
		hiveInit(m_pCloud, m_pRetouchConfig);

		m_pHoleRepairerConfig = new CPointCloudRetouchConfig;
		ASSERT_EQ(hiveConfig::hiveParseConfig(HoleRepairerConfigFile, hiveConfig::EConfigType::XML, m_pHoleRepairerConfig), hiveConfig::EParseResult::SUCCEED);
		m_Repairer.init(m_pHoleRepairerConfig);

		//读入所有boundary points
		for (int i = 1; hiveUtility::hiveLocateFile(DataPath + ModelNames[m_TestNumber] + std::to_string(i) + ".txt") != ""; i++)
			m_BoundaryIndices.push_back(_loadIndices(DataPath + ModelNames[m_TestNumber] + std::to_string(i) + ".txt"));
		ASSERT_TRUE(!m_BoundaryIndices.empty());
		if (hiveUtility::hiveLocateFile(DataPath + ModelNames[m_TestNumber] + "_input.txt") != "")
			m_InputIndices = _loadIndices(DataPath + ModelNames[m_TestNumber] + "_input.txt");
		ASSERT_TRUE(!m_InputIndices.empty());

		if (ENABLE_VISUALIZER)
		{
			m_pVisualizer = hiveObliquePhotography::Visualization::CPointCloudVisualizer::getInstance();
			m_pVisualizer->init(m_pCloud, false);
			std::vector<std::size_t> Label;
			hiveDumpPointLabel(Label);
			m_pVisualizer->refresh(Label);
			m_pPCLVisualizer = hiveObliquePhotography::Visualization::hiveGetPCLVisualizer();

			for (auto& Indices : m_BoundaryIndices)
			{
				std::vector<pcl::PointXYZRGBNormal> TempPoints;
				m_Repairer.repairHoleByBoundaryAndInput(Indices, m_InputIndices, TempPoints);
				PointCloud_t::Ptr TempCloud(new PointCloud_t);
				for (auto& Point : TempPoints)
				{
					if (Point.r == 0 && Point.g == 0 && Point.b == 0)
						Point.rgb = -1;
					else
					{
						//注释取消高亮
						Point.r = 255;
						Point.g = 0;
						Point.b = 0;
					}
					TempCloud->push_back(Point);
				}

				auto TempBox = _getBoundingBox(m_pCloud, Indices);
				auto AxisOrder = _getAxisOrder(m_Repairer.calcPlane(Indices, std::get<0>(TempBox)));
				auto X = AxisOrder[0], Y = AxisOrder[1], Z = AxisOrder[2];

				const float BoxHeight = 0.5f;	//包围盒的高度不会使用
				std::get<1>(TempBox).data()[Z] -= BoxHeight;
				std::get<2>(TempBox).data()[Z] += BoxHeight;

				//add lattices
				const int PointSize = 3;
				m_pPCLVisualizer->addPointCloud<pcl::PointXYZRGBNormal>(TempCloud, "TempCloud" + std::to_string(TempCloud->size()) + std::to_string(Indices.size()));
				m_pPCLVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PointSize, "TempCloud" + std::to_string(TempCloud->size()) + std::to_string(Indices.size()));
				//add box line
				_drawBox(std::make_pair(std::get<1>(TempBox), std::get<2>(TempBox)));
			}
		}
	}

	void TearDown() override
	{
		if (ENABLE_VISUALIZER)
		{
			if (m_pVisualizer)
				m_pVisualizer->run();
		}

		delete m_pRetouchConfig;
		delete m_pHoleRepairerConfig;
	}

	std::vector<int> _loadIndices(const std::string& vPath)
	{
		std::vector<int> Indices;
		std::ifstream File(vPath);
		boost::archive::text_iarchive ia(File);
		ia >> BOOST_SERIALIZATION_NVP(Indices);
		File.close();
		return Indices;
	}

	std::tuple<Eigen::Matrix3f, Eigen::Vector3f, Eigen::Vector3f> _getBoundingBox(PointCloud_t::Ptr vCloud, const std::vector<pcl::index_t>& vIndices)
	{
		std::vector<Eigen::Vector3f> RawPosSet;
		auto pManager = CPointCloudRetouchManager::getInstance();
		pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>);
		for (auto Index : vIndices)
		{
			auto TempPos = vCloud->points[Index].getVector3fMap();
			pcl::PointXYZ TempPoint;
			TempPoint.x = TempPos.x(); TempPoint.y = TempPos.y(); TempPoint.z = TempPos.z();
			pCloud->push_back(TempPoint);
			RawPosSet.push_back(Eigen::Vector3f{ TempPos.x(), TempPos.y(), TempPos.z() });
		}
		Eigen::Matrix3f CovarianceMatrix;
		Eigen::Vector4f Centroid;
		pcl::compute3DCentroid(*pCloud, Centroid);
		pcl::computeCovarianceMatrix(*pCloud, Centroid, CovarianceMatrix);

		Eigen::EigenSolver<Eigen::Matrix3f> EigenMat(CovarianceMatrix);
		Eigen::Vector3f EigenValue = EigenMat.pseudoEigenvalueMatrix().diagonal();
		Eigen::Matrix3f EigenVector = EigenMat.pseudoEigenvectors();

		std::vector<std::tuple<float, Eigen::Vector3f>> EigenValueAndVector;
		int Size = static_cast<int>(EigenValue.size());
		EigenValueAndVector.reserve(Size);
		for (int i = 0; i < Size; ++i)
			EigenValueAndVector.push_back(std::tuple<float, Eigen::Vector3f>(EigenValue[i], EigenVector.col(i)));
		std::ranges::sort(EigenValueAndVector,
			[&](const std::tuple<float, Eigen::Vector3f>& a, const std::tuple<float, Eigen::Vector3f>& b) -> bool {
				return std::get<0>(a) > std::get<0>(b);
			});
		for (int i = 0; i < Size; ++i)
		{
			EigenVector.col(i).swap(std::get<1>(EigenValueAndVector[i]));
		}

		Eigen::Vector3f Min{ FLT_MAX, FLT_MAX, FLT_MAX };
		Eigen::Vector3f Max{ -FLT_MAX, -FLT_MAX, -FLT_MAX };
		auto update = [&](const Eigen::Vector3f& vPos)
		{
			for (int i = 0; i < 3; i++)
			{
				if (vPos.data()[i] < Min.data()[i])
					Min.data()[i] = vPos.data()[i];
				if (vPos.data()[i] > Max.data()[i])
					Max.data()[i] = vPos.data()[i];
			}
		};

		for (auto& Pos : RawPosSet)
		{
			Eigen::Vector3f AfterPos = EigenVector * Pos;
			update(AfterPos);
		}

		std::tuple<Eigen::Matrix3f, Eigen::Vector3f, Eigen::Vector3f> ObbBox(EigenVector, Min, Max);
		return ObbBox;
	}
	
	std::vector<int> _getAxisOrder(const Eigen::Vector4f& vPlane)
	{
		Eigen::Vector3f PlaneNormal{ vPlane.x(), vPlane.y(), vPlane.z() };
		auto MinAxis = PlaneNormal.maxCoeff();
		std::vector<int> AxisOrder(3);	//Min在最后
		if (MinAxis == PlaneNormal.x())
			AxisOrder = { 1, 2, 0 };
		else if (MinAxis == PlaneNormal.y())
			AxisOrder = { 2, 0, 1 };
		else if (MinAxis == PlaneNormal.z())
			AxisOrder = { 0, 1, 2 };
		return AxisOrder;
	}

	bool _isInBox(const pcl::PointXYZRGBNormal& vPoint, const std::pair<Eigen::Vector3f, Eigen::Vector3f>& vBox)
	{
		Eigen::Vector3f Pos;
		Pos.x() = vPoint.x;
		Pos.y() = vPoint.y;
		Pos.z() = vPoint.z;
		for (int i = 0; i < 3; i++)
		{
			if (Pos.data()[i] < vBox.first.data()[i])
				return false;
			if (Pos.data()[i] > vBox.second.data()[i])
				return false;
		}
		return true;
	}
	bool _isInBox(const Eigen::Vector3f vPos, const std::tuple<Eigen::Matrix3f, Eigen::Vector3f, Eigen::Vector3f>& vBox)
	{
		Eigen::Vector3f TempPos = std::get<0>(vBox) * vPos;
		for (int i = 0; i < 3; i++)
		{
			if (TempPos.data()[i] < std::get<1>(vBox).data()[i])
				return false;
			if (TempPos.data()[i] > std::get<2>(vBox).data()[i])
				return false;
		}
		return true;
	}

	//test generation
	void _testLatticesGeneration(const std::vector<int>& vIndices)
	{
		SPlaneInfos PlaneInfos;
		std::vector<std::vector<SLattice>> PlaneLattices;
		const std::vector<Eigen::Vector2i> ResolutionSet = { { 1, 1 }, {32, 32}, {16, 9}, {7, 101} };
        auto Box = _getBoundingBox(m_pCloud, vIndices);
		auto Plane = m_Repairer.calcPlane(vIndices, std::get<0>(Box));
		
		std::vector<size_t> AxisOrder { 0, 1, 2 };
		auto X = AxisOrder[0], Y = AxisOrder[1], Z = AxisOrder[2];
		const float BoxHeight = 0.5f;
		//排除分辨率x, y顺序影响
		for (auto& Resolution : ResolutionSet)
		{
			m_Repairer.generateLattices(Plane, Box, Resolution, PlaneInfos, PlaneLattices);
			ASSERT_EQ(PlaneLattices.size(), Resolution.y());
			for (auto& PerRow : PlaneLattices)
				ASSERT_EQ(PerRow.size(), Resolution.x());

			std::get<1>(PlaneInfos.BoundingBox).data()[Z] -= BoxHeight;
			std::get<2>(PlaneInfos.BoundingBox).data()[Z] += BoxHeight;
			//盒内和平面上
			for (int Y = 0; Y < Resolution.y(); Y++)
				for (int X = 0; X < Resolution.x(); X++)
				{
					auto& Pos = PlaneLattices[Y][X].CenterPos;

					Eigen::Vector3f RestorePos = std::get<0>(Box).inverse() * Pos;
					ASSERT_TRUE(_isInBox(RestorePos, PlaneInfos.BoundingBox));
					float Point2Plane = Pos.x() * Plane.x() + Pos.y() * Plane.y() + Pos.z() * Plane.z() + Plane.w();
					ASSERT_NEAR(Point2Plane, 0, Epsilon);
				}

			//格子大小均匀
			const std::vector<Eigen::Vector2i> CoordSet{ {2, 4}, {3, 5}, {11, 6} };
			for (auto& Coord : CoordSet)
			{
				if (Resolution.x() > Coord.x() && Resolution.y() > Coord.y())
				{
					Eigen::Vector3f DeltaPos = PlaneLattices[Coord.y()][Coord.x()].CenterPos - PlaneLattices[0][0].CenterPos;
					ASSERT_NEAR(DeltaPos.data()[X], Coord.x() * PlaneInfos.LatticeSize.x(), Epsilon);
					ASSERT_NEAR(DeltaPos.data()[Y], Coord.y() * PlaneInfos.LatticeSize.y(), Epsilon);
				}
			}
		}
	}

	//test projection
	void _testPointsProjection(const std::vector<int>& vIndices)
	{
		SPlaneInfos PlaneInfos;
		std::vector<std::vector<SLattice>> PlaneLattices;
		auto Box = _getBoundingBox(m_pCloud, vIndices);
		auto Plane = m_Repairer.calcPlane(vIndices,std::get<0>(Box));
		std::vector<size_t> AxisOrder { 0, 1, 2 };
		auto X = AxisOrder[0], Y = AxisOrder[1], Z = AxisOrder[2];
		const Eigen::Vector2i Resolution{ 16, 16 };
		m_Repairer.generateLattices(Plane, Box, Resolution, PlaneInfos, PlaneLattices);
		m_Repairer.projectPoints({}, PlaneInfos, PlaneLattices);
		
		for (int Y = 0; Y < Resolution.y(); Y++)
			for (int X = 0; X < Resolution.x(); X++)
			{
				auto& Lattice = PlaneLattices[Y][X];
				for (auto Index : Lattice.Indices)
				{
					Eigen::Vector3f PointPos{ m_pCloud->points[Index].x, m_pCloud->points[Index].y, m_pCloud->points[Index].z };
					ASSERT_LE(abs(PlaneInfos.Normal.dot(PointPos - Lattice.CenterPos)), 0.99f);
				}
			}
	}

	//test filling
	void _testOriginInfosFilling(const std::vector<int>& vIndices)
	{
		SPlaneInfos PlaneInfos;
		std::vector<std::vector<SLattice>> PlaneLattices;
		auto Box = _getBoundingBox(m_pCloud, vIndices);
		auto Plane = m_Repairer.calcPlane(vIndices, std::get<0>(Box));
		std::vector<size_t> AxisOrder{ 0, 1, 2 };
		auto X = AxisOrder[0], Y = AxisOrder[1], Z = AxisOrder[2];
		const Eigen::Vector2i Resolution{ 16, 16 };
		m_Repairer.generateLattices(Plane, Box, Resolution, PlaneInfos, PlaneLattices);
		m_Repairer.projectPoints({}, PlaneInfos, PlaneLattices);

		const int KernelSize = 3;
		const int Delta = KernelSize / 2;

		for (int Y = 0; Y < Resolution.y(); Y++)
			for (int X = 0; X < Resolution.x(); X++)
			{
				auto& Lattice = PlaneLattices[Y][X];
				if (!Lattice.Indices.empty())
				{
					ASSERT_NE(Lattice.Color.norm(), 0);
					ASSERT_NE(Lattice.Height(0, 0), 0.0f);
				}
				else    //空的若周围都有点(非空洞部分)，也得有值
				{
					std::size_t NumValue = 0;
					for (int i = Y - Delta; i <= Y + Delta; i++)
						for (int k = X - Delta; k <= X + Delta; k++)
							if (i >= 0 && i < Resolution.y() && k >= 0 && k < Resolution.x())
								if (!PlaneLattices[i][k].Indices.empty())
									NumValue++;
					if (NumValue > pow(KernelSize, 2) * 2 / 3)
					{
						EXPECT_NE(Lattice.Color.norm(), 0);
						EXPECT_NE(Lattice.Height(0, 0), 0.0f);
					}
				}
			}
		//auto ColorMatrix = m_Repairer.extractMatrix<Eigen::Vector3i>(PlaneLattices, offsetof(SLattice, Color));
		//m_Repairer.outputImage(ColorMatrix, "Color_" + std::to_string(PlaneLattices[0][0].Color.x()) + std::to_string(PlaneLattices[0][0].Color.y()) + std::to_string(PlaneLattices[0][0].Color.z()) + ".png");
	}

	void _drawBox(const std::pair<Eigen::Vector3f, Eigen::Vector3f>& vBox)
	{
		static int Id = -1;
		Id++;
		std::vector<Eigen::Vector3f> Box;
		Box.push_back(vBox.first);
		Box.push_back(vBox.second);
		std::vector<pcl::PointXYZ> BoxEndPoints;
		for (int i = 0; i < 2; i++)
			for (int k = 0; k < 2; k++)
				for (int m = 0; m < 2; m++)
				{
					pcl::PointXYZ TempPoint;
					TempPoint.x = Box[i].x();
					TempPoint.y = Box[k].y();
					TempPoint.z = Box[m].z();
					BoxEndPoints.push_back(TempPoint);
				}

		static const std::vector<std::pair<int, int>> CubeIndex =
		{
			{0, 1},{2, 3},{4, 5},{6, 7},
			{0, 2},{1, 3},{4, 6},{5, 7},
			{0, 4},{1, 5},{7, 3},{2, 6}
		};
		for (auto Pair : CubeIndex)
			m_pPCLVisualizer->addLine(BoxEndPoints[Pair.first], BoxEndPoints[Pair.second], "Line" + std::to_string(Id) + std::to_string(Pair.first) + std::to_string(Pair.second));
	}

	hiveConfig::CHiveConfig* m_pRetouchConfig = nullptr;
	hiveConfig::CHiveConfig* m_pHoleRepairerConfig = nullptr;
	PointCloud_t::Ptr m_pCloud = nullptr;
	hiveObliquePhotography::Visualization::CPointCloudVisualizer* m_pVisualizer = nullptr;
	pcl::visualization::PCLVisualizer* m_pPCLVisualizer = nullptr;

	CHoleRepairer m_Repairer;

	std::vector<std::vector<int>> m_BoundaryIndices;
	std::vector<int> m_InputIndices;
	static int m_TestNumber;
private:
};
int TestLatticesProjection::m_TestNumber = -1;

TEST_F(TestLatticesProjection, Boundary_Detection_BaseTest_1)
{
	auto& Boundary = m_BoundaryIndices.front();
	_testLatticesGeneration(Boundary);
	_testPointsProjection(Boundary);
	_testOriginInfosFilling(Boundary);
}

TEST_F(TestLatticesProjection, Boundary_Detection_BaseTest_2)
{
	for (auto& Boundary : m_BoundaryIndices)
	{
		_testLatticesGeneration(Boundary);
		_testPointsProjection(Boundary);
		_testOriginInfosFilling(Boundary);
	}
}

