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
#include <pcl/io/pcd_io.h>

//测试用例列表：
// LatticesProjectionBaseTest:通过不同空洞边界点正确生成对应栅格，并将边界点投影到正确栅格内；
//	* Lattices_Projection_BaseTest_1:整个场景一个小空洞；
//  * Lattices_Projection_BaseTest_2:场景有五个空洞；

#define ENABLE_VISUALIZER true

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

		m_pHoleRepairerConfig = new CPointCloudRetouchConfig;
		ASSERT_EQ(hiveConfig::hiveParseConfig(HoleRepairerConfigFile, hiveConfig::EConfigType::XML, m_pHoleRepairerConfig), hiveConfig::EParseResult::SUCCEED);

		m_pCloud.reset(new PointCloud_t);
		m_pCloud = hiveObliquePhotography::hiveInitPointCloudScene({ DataPath + ModelNames[m_TestNumber] + ".ply" });
		hiveInit(m_pCloud, m_pRetouchConfig);

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

	std::pair<Eigen::Vector3f, Eigen::Vector3f> _getBoundingBox(PointCloud_t::Ptr vCloud, const std::vector<pcl::index_t>& vIndices)
	{
		Eigen::Vector3f Min{ FLT_MAX, FLT_MAX, FLT_MAX };
		Eigen::Vector3f Max{ -FLT_MAX, -FLT_MAX, -FLT_MAX };

		for (auto Index : vIndices)
		{
			Eigen::Vector3f Pos;
			Pos.x() = vCloud->points[Index].x;
			Pos.y() = vCloud->points[Index].y;
			Pos.z() = vCloud->points[Index].z;
			for (int i = 0; i < 3; i++)
			{
				if (Pos.data()[i] < Min.data()[i])
					Min.data()[i] = Pos.data()[i];
				if (Pos.data()[i] > Max.data()[i])
					Max.data()[i] = Pos.data()[i];
			}
		}
		return { Min, Max };
	}
	
	bool _isInBox(const pcl::PointSurfel& vPoint, const std::pair<Eigen::Vector3f, Eigen::Vector3f>& vBox)
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

	std::vector<std::vector<int>> m_BoundaryIndices;
	std::vector<int> m_InputIndices;
	static int m_TestNumber;
private:
};
int TestLatticesProjection::m_TestNumber = -1;

TEST_F(TestLatticesProjection, Boundary_Detection_BaseTest_1)
{
	CHoleRepairer Repairer;
	Repairer.init(m_pHoleRepairerConfig);
	for (auto& Indices : m_BoundaryIndices)
	{
		std::vector<pcl::PointSurfel> TempPoints;
		Repairer.repairHoleByBoundaryAndInput(Indices, m_InputIndices, TempPoints);
		PointCloud_t::Ptr TempCloud(new PointCloud_t);
		for (auto& Point : TempPoints)
		{
			if (Point.r == 0 && Point.g == 0 && Point.b == 0)
				Point.rgba = -1;
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
		auto BoxLength = TempBox.second - TempBox.first;
		Eigen::Vector3f PlaneNormal{ BoxLength.x(), BoxLength.y(), BoxLength.z() };
		auto MinAxis = PlaneNormal.minCoeff();
		std::vector<std::size_t> AxisOrder(3);	//Min在最后
		if (MinAxis == PlaneNormal.x())
			AxisOrder = { 1, 2, 0 };
		else if (MinAxis == PlaneNormal.y())
			AxisOrder = { 2, 0, 1 };
		else if (MinAxis == PlaneNormal.z())
			AxisOrder = { 0, 1, 2 };
		auto X = AxisOrder[0], Y = AxisOrder[1], Z = AxisOrder[2];

		const float BoxHeight = 0.5f;	//包围盒的高度不会使用
		TempBox.first.data()[Z] -= BoxHeight;
		TempBox.second.data()[Z] += BoxHeight;
		for (auto& Point : TempPoints)
			EXPECT_TRUE(_isInBox(Point, TempBox));

		if (ENABLE_VISUALIZER)
		{
			//add lattices
			const int PointSize = 3;
			m_pPCLVisualizer->addPointCloud<pcl::PointSurfel>(TempCloud, "TempCloud" + std::to_string(Indices.size()));
			m_pPCLVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PointSize, "TempCloud" + std::to_string(Indices.size()));
			//add box line
			_drawBox(TempBox);
		}
	}
}

TEST_F(TestLatticesProjection, Boundary_Detection_BaseTest_2)
{
	CHoleRepairer Repairer;
	Repairer.init(m_pHoleRepairerConfig);
	for (auto& Indices : m_BoundaryIndices)
	{
		std::vector<pcl::PointSurfel> TempPoints;
		Repairer.repairHoleByBoundaryAndInput(Indices, m_InputIndices, TempPoints);
		PointCloud_t::Ptr TempCloud(new PointCloud_t);
		for (auto& Point : TempPoints)
		{
			if (Point.r == 0 && Point.g == 0 && Point.b == 0)
				Point.rgba = -1;
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
		auto BoxLength = TempBox.second - TempBox.first;
		Eigen::Vector3f PlaneNormal{ BoxLength.x(), BoxLength.y(), BoxLength.z() };
		auto MinAxis = PlaneNormal.minCoeff();
		std::vector<std::size_t> AxisOrder(3);	//Min在最后
		if (MinAxis == PlaneNormal.x())
			AxisOrder = { 1, 2, 0 };
		else if (MinAxis == PlaneNormal.y())
			AxisOrder = { 2, 0, 1 };
		else if (MinAxis == PlaneNormal.z())
			AxisOrder = { 0, 1, 2 };
		auto X = AxisOrder[0], Y = AxisOrder[1], Z = AxisOrder[2];

		const float BoxHeight = 0.5f;	//包围盒的高度不会使用
		TempBox.first.data()[Z] -= BoxHeight;
		TempBox.second.data()[Z] += BoxHeight;
		for (auto& Point : TempPoints)
			EXPECT_TRUE(_isInBox(Point, TempBox));

		if (ENABLE_VISUALIZER)
		{
			//add lattices
			const int PointSize = 3;
			m_pPCLVisualizer->addPointCloud<pcl::PointSurfel>(TempCloud, "TempCloud" + std::to_string(Indices.size()));
			m_pPCLVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PointSize, "TempCloud" + std::to_string(Indices.size()));
			//add box line
			_drawBox(TempBox);
		}
	}
}

