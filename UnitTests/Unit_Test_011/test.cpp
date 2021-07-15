#include "pch.h"
#include "ObliquePhotographyDataInterface.h"
#include "PointCloudRetouchInterface.h"
#include "PointCloudRetouchManager.h"
#include "PointCloudRetouchConfig.h"
#include "VisualizationInterface.h"
#include "InitialClusterCreator.h"
#include "PointCluster.h"
//主要围绕CInitialClusterCreator::createInitialCluster()进行测试

//测试用例列表：
//  * BaseTest1_Create_Cluster: 给定圆内点集及不同硬度，能创建出具有分界的GenerationSet, ValidationSet的CPointCluster

//	* DeathTest1_User_Caused_Error: 给定用户导致的错误：空索引集，错误的硬度，错误的label，能不抛出任何异常及改变其他类属性
//	* DeathTest2_User_Caused_Error: 给定程序导致的错误：错误索引集，错误PV矩阵，错误config，能抛出异常

using namespace hiveObliquePhotography;

#define PI 3.141592653589793
#define radians(x) (x * PI / 180)

const std::string ConfigPath = "../TestData/Config/Test011_PointCloudRetouchConfig.xml";
const std::string CameraPath = "../TestData/Test011_Model/VirtualCircleCameraInfo.txt";

class CTestCreateInitialCluster : public testing::Test
{ 
protected:
	void SetUp() override
	{
		m_pCloud.reset(new PointCloud_t);	
		float AngleStep = 10.0f, RadiusStep = 0.1f;
		for (float Angle = 0.0f; Angle < 360.0f; Angle+= AngleStep)
		{
			for (float Radius = 0.1f; Radius <= m_CircleRadius; Radius += RadiusStep)
			{
				pcl::PointSurfel Point;
				Point.x = Radius * cos(radians(Angle));
				Point.y = Radius * sin(radians(Angle));
				Point.z = -3.0f;
				Point.curvature = Radius;
				Point.rgba = -1;
				m_pCloud->push_back(Point);
			}
		}
		for (int i = 0; i < m_pCloud->size(); i++)
			m_Indices.push_back(i);

		m_pConfig = new PointCloudRetouch::CPointCloudRetouchConfig;
		if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, m_pConfig) != hiveConfig::EParseResult::SUCCEED)
		{
			_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
			return;
		}

		PointCloudRetouch::hiveInit(m_pCloud, m_pConfig);

		m_pVisualizer.reset(new pcl::visualization::PCLVisualizer);
		m_pVisualizer->addPointCloud<pcl::PointSurfel>(m_pCloud, "Cloud2Show");
		m_pVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud2Show");
		m_pVisualizer->loadCameraParameters(CameraPath);
		m_pVisualizer->updateCamera();

		pcl::visualization::Camera Camera;
		m_pVisualizer->getCameraParameters(Camera);
		Eigen::Matrix4d Proj, View;
		Camera.computeProjectionMatrix(Proj);
		Camera.computeViewMatrix(View);
		m_PV = Proj * View;

	}

	void TearDown() override
	{
		while (!m_pVisualizer->wasStopped())
		{
			m_pVisualizer->spinOnce(16);
		}
		delete m_pConfig;
	}

	PointCloud_t::Ptr m_pCloud = nullptr;
	hiveConfig::CHiveConfig* m_pConfig = nullptr;
	pcl::visualization::PCLVisualizer::Ptr m_pVisualizer = nullptr;

	std::vector<int> m_Indices;
	Eigen::Matrix4d m_PV;

	float m_CircleRadius = 1.0f;

	PointCloudRetouch::CInitialClusterCreator m_Creator;
};

//给定圆内点集及不同硬度，能创建出具有分界的GenerationSet, ValidationSet的CPointCluster
TEST_F(CTestCreateInitialCluster, BaseTest1_Create_Cluster)
{
	//无硬度
	{
		float ZeroHardness = 0.0f;
		PointCloudRetouch::CPointCluster* pCluster = nullptr;
		EXPECT_NO_THROW(pCluster = m_Creator.createInitialCluster(m_Indices, m_PV, ZeroHardness, PointCloudRetouch::EPointLabel::UNWANTED, PointCloudRetouch::CPointCloudRetouchManager::getInstance()->getClusterConfig(true)));
		EXPECT_NE(pCluster, nullptr);
		if (pCluster)
		{
			auto GenerationSet = pCluster->getCoreRegion();
			EXPECT_TRUE(GenerationSet.empty());
		}
	}

	////硬度满
	//{
	//	float FullHardness = 1.0f;
	//	auto pCluster = m_Creator.createInitialCluster(m_Indices, m_PV, FullHardness, PointCloudRetouch::EPointLabel::UNWANTED, PointCloudRetouch::CPointCloudRetouchManager::getInstance()->getClusterConfig(true));
	//	EXPECT_NE(pCluster, nullptr);
	//	auto GenerationSet = pCluster->getCoreRegion();
	//	EXPECT_EQ(GenerationSet.size(), m_Indices.size());
	//}

	////非特殊情况
	//for (float Hardness = 0.1f; Hardness < 1.0f; Hardness += 0.2f)
	//{
	//	auto pCluster = m_Creator.createInitialCluster(m_Indices, m_PV, Hardness, PointCloudRetouch::EPointLabel::UNWANTED, PointCloudRetouch::CPointCloudRetouchManager::getInstance()->getClusterConfig(true));
	//	EXPECT_NE(pCluster, nullptr);
	//	auto GenerationSet = pCluster->getCoreRegion();

	//	//找GenerationSet内最大距离
	//	float MaxGenerationRadius = -FLT_MAX;
	//	for (auto Index : GenerationSet)
	//	{
	//		auto Temp = m_pCloud->points[Index].curvature;
	//		if (Temp > MaxGenerationRadius)
	//			MaxGenerationRadius = Temp;
	//	}
	//	EXPECT_NE(MaxGenerationRadius, m_CircleRadius);
	//	EXPECT_NE(GenerationSet.size(), m_Indices.size());

	//	//GenerationSet里边界一圈上的点都得在
	//	int NumGenerationBoundPoints = 0;
	//	for (auto Index : GenerationSet)
	//	{
	//		auto Temp = m_pCloud->points[Index].curvature;
	//		if (Temp == MaxGenerationRadius)
	//			NumGenerationBoundPoints++;
	//	}
	//	EXPECT_NE(NumGenerationBoundPoints, 0);
	//	EXPECT_LE(NumGenerationBoundPoints, m_Indices.size());

	//	//ValidationSet的点必须距离都更大
	//	std::vector<int> ValidationSet;
	//	std::set_difference(GenerationSet.begin(), GenerationSet.end(),
	//		m_Indices.begin(), m_Indices.end(),
	//		std::inserter(ValidationSet, ValidationSet.begin()));
	//	EXPECT_TRUE(!ValidationSet.empty());

	//	float MinValidationRadius = FLT_MAX;
	//	for (auto Index : ValidationSet)
	//	{
	//		auto Temp = m_pCloud->points[Index].curvature;
	//		EXPECT_GT(Temp, MaxGenerationRadius);

	//		if (Temp < MinValidationRadius)
	//			MinValidationRadius = Temp;
	//	}

	//	//ValidationSet边界一圈上的点都得在
	//	int NumValidationBoundPoints = 0;
	//	for (auto Index : ValidationSet)
	//	{
	//		auto Temp = m_pCloud->points[Index].curvature;
	//		if (Temp == MinValidationRadius)
	//			NumValidationBoundPoints++;
	//	}
	//	EXPECT_NE(NumValidationBoundPoints, 0);
	//	EXPECT_LE(NumValidationBoundPoints, m_Indices.size());
	//}

	////KEPT也能成功创建
	//auto pCluster = m_Creator.createInitialCluster(m_Indices, m_PV, 0.5f, PointCloudRetouch::EPointLabel::KEPT, PointCloudRetouch::CPointCloudRetouchManager::getInstance()->getClusterConfig(false));
	//EXPECT_NE(pCluster, nullptr);
}

//空索引集，错误的硬度，错误的label(如创建DISCARDED的cluster)
TEST_F(CTestCreateInitialCluster, DeathTest1_User_Caused_Error)
{
	//空索引
	std::vector<int> EmptyIndices;
	EXPECT_NO_THROW(m_Creator.createInitialCluster(EmptyIndices, m_PV, 0.5f, PointCloudRetouch::EPointLabel::UNWANTED, PointCloudRetouch::CPointCloudRetouchManager::getInstance()->getClusterConfig(true)));
	
	//负硬度和超过1的
	float MinusHardness = -1.0f;
	EXPECT_NO_THROW(m_Creator.createInitialCluster(m_Indices, m_PV, MinusHardness, PointCloudRetouch::EPointLabel::UNWANTED, PointCloudRetouch::CPointCloudRetouchManager::getInstance()->getClusterConfig(true)));
	float BigHardness = 2.0f;
	EXPECT_NO_THROW(m_Creator.createInitialCluster(m_Indices, m_PV, BigHardness, PointCloudRetouch::EPointLabel::UNWANTED, PointCloudRetouch::CPointCloudRetouchManager::getInstance()->getClusterConfig(true)));

	//错误Label: Discard, Undetermined
	EXPECT_NO_THROW(m_Creator.createInitialCluster(m_Indices, m_PV, 0.5f, PointCloudRetouch::EPointLabel::DISCARDED, PointCloudRetouch::CPointCloudRetouchManager::getInstance()->getClusterConfig(true)));
	EXPECT_NO_THROW(m_Creator.createInitialCluster(m_Indices, m_PV, 0.5f, PointCloudRetouch::EPointLabel::UNDETERMINED, PointCloudRetouch::CPointCloudRetouchManager::getInstance()->getClusterConfig(true)));
}

//错误索引集，错误PV矩阵，错误config
TEST_F(CTestCreateInitialCluster, DeathTest2_User_Caused_Error)
{
	//带负数
	std::vector<int> MinusIndices;
	MinusIndices.push_back(-1);
	EXPECT_ANY_THROW(m_Creator.createInitialCluster(MinusIndices, m_PV, 0.5f, PointCloudRetouch::EPointLabel::UNWANTED, PointCloudRetouch::CPointCloudRetouchManager::getInstance()->getClusterConfig(true)));

	//超区域
	std::vector<int> GreatIndices;
	GreatIndices.push_back(m_Indices.size());
	EXPECT_ANY_THROW(m_Creator.createInitialCluster(GreatIndices, m_PV, 0.5f, PointCloudRetouch::EPointLabel::UNWANTED, PointCloudRetouch::CPointCloudRetouchManager::getInstance()->getClusterConfig(true)));

	//空PV矩阵
	EXPECT_ANY_THROW(m_Creator.createInitialCluster(m_Indices, Eigen::Matrix4d{}, 0.5f, PointCloudRetouch::EPointLabel::UNWANTED, PointCloudRetouch::CPointCloudRetouchManager::getInstance()->getClusterConfig(true)));

	//空config
	EXPECT_ANY_THROW(m_Creator.createInitialCluster(m_Indices, m_PV, 0.5f, PointCloudRetouch::EPointLabel::UNWANTED, nullptr));

}