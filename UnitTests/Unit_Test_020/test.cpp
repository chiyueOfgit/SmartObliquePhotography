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

using namespace hiveObliquePhotography::PointCloudRetouch;

const auto RetouchConfigFile = TESTMODEL_DIR + std::string("Config/Test018_PointCloudRetouchConfig.xml");
const auto DataPath = TESTMODEL_DIR + std::string("Test018_Model/");

const std::vector<std::string> ModelNames{ "one_hole", "five_holes" };

class TestPlaneAndProject : public testing::Test
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

		//读入所有boundary points
		for (int i = 1; hiveUtility::hiveLocateFile(DataPath + ModelNames[m_TestNumber] + std::to_string(i) + ".txt") != ""; i++)
			m_BoundaryIndices.push_back(_loadIndices(DataPath + ModelNames[m_TestNumber] + std::to_string(i) + ".txt"));
		ASSERT_TRUE(!m_BoundaryIndices.empty());
	}

	void TearDown() override
	{
		delete m_pRetouchConfig;
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

	hiveConfig::CHiveConfig* m_pRetouchConfig = nullptr;
	PointCloud_t::Ptr m_pCloud = nullptr;

	std::vector<std::vector<int>> m_BoundaryIndices;
	static int m_TestNumber;
private:
};
int TestPlaneAndProject::m_TestNumber = -1;

TEST_F(TestPlaneAndProject, Boundary_Detection_BaseTest_1)
{
	auto pVisualizer = hiveObliquePhotography::Visualization::CPointCloudVisualizer::getInstance();
	pVisualizer->init(m_pCloud, false);
	std::vector<std::size_t> Label;
	hiveDumpPointLabel(Label);
	pVisualizer->refresh(Label);

	CHoleRepairer Repairer;
	auto pPCLVisualizer = hiveObliquePhotography::Visualization::hiveGetPCLVisualizer();
	for (auto& Indices : m_BoundaryIndices)
	{
		std::vector<pcl::PointSurfel> TempPoints;
		Repairer.repairHoleByBoundaryAndInput(Indices, {1}, TempPoints, nullptr);
		PointCloud_t::Ptr TempCloud(new PointCloud_t);
		for (auto& Point : TempPoints)
		{
			Point.rgba = -1;
			TempCloud->push_back(Point);
		}
		pPCLVisualizer->addPointCloud<pcl::PointSurfel>(TempCloud, "TempCloud" + std::to_string(Indices.size()));

		//add box line
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

		const float BoxHeight = 0.5f;	//可视化包围盒的高度(原本太矮)
		TempBox.first.data()[Z] -= BoxHeight;
		TempBox.second.data()[Z] += BoxHeight;
		std::vector<Eigen::Vector3f> Box;
		Box.push_back(TempBox.first);
		Box.push_back(TempBox.second);
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

		for (int i = 0; i < BoxEndPoints.size(); i++)
			for (int k = i; k < BoxEndPoints.size(); k++)
				if (k != i)
					pPCLVisualizer->addLine(BoxEndPoints[i], BoxEndPoints[k], "Line" + std::to_string(Indices.size()) + std::to_string(i) + std::to_string(k));
	}

	pVisualizer->run();
}

TEST_F(TestPlaneAndProject, Boundary_Detection_BaseTest_2)
{
	auto pVisualizer = hiveObliquePhotography::Visualization::CPointCloudVisualizer::getInstance();
	pVisualizer->init(m_pCloud, false);
	std::vector<std::size_t> Label;
	hiveDumpPointLabel(Label);
	pVisualizer->refresh(Label);

	CHoleRepairer Repairer;
	auto pPCLVisualizer = hiveObliquePhotography::Visualization::hiveGetPCLVisualizer();
	for (auto& Indices : m_BoundaryIndices)
	{
		std::vector<pcl::PointSurfel> TempPoints;
		Repairer.repairHoleByBoundaryAndInput(Indices, { 1 }, TempPoints, nullptr);
		PointCloud_t::Ptr TempCloud(new PointCloud_t);
		for (auto& Point : TempPoints)
		{
			Point.rgba = -1;
			TempCloud->push_back(Point);
		}
		pPCLVisualizer->addPointCloud<pcl::PointSurfel>(TempCloud, "TempCloud" + std::to_string(Indices.size()));

		//add box line
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

		const float BoxHeight = 0.5f;	//可视化包围盒的高度(原本太矮)
		TempBox.first.data()[Z] -= BoxHeight;
		TempBox.second.data()[Z] += BoxHeight;
		std::vector<Eigen::Vector3f> Box;
		Box.push_back(TempBox.first);
		Box.push_back(TempBox.second);
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

		for (int i = 0; i < BoxEndPoints.size(); i++)
			for (int k = i; k < BoxEndPoints.size(); k++)
				if (k != i)
					pPCLVisualizer->addLine(BoxEndPoints[i], BoxEndPoints[k], "Line" + std::to_string(Indices.size()) + std::to_string(i) + std::to_string(k));
	}

	pVisualizer->run();
}

