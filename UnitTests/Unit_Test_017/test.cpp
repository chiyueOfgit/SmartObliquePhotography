#include "pch.h"

#include <common/MathInterface.h>
#include "PointCloudRetouchInterface.h"
#include "PointCloudRetouchConfig.h"
#include <fstream>
#include <boost/archive/text_oarchive.hpp> 
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <pcl/io/pcd_io.h>

#include "PointCloudRetouchManager.h"
#include "PointSetPreprocessor.h"

#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/visualization/common/common.h"

//测试用例列表：
//SelectingBaseTest:给出明显的判别逻辑情况，证明选择的正确性；
//  * Selecting_NoThroughTest_CompleteTree:选取区域完全属于一棵树；
//  * Selecting_NoThroughTest_CompleteGround:选取区域完全属于地面；
//  * Selecting_NoThroughTest_CompleteBuilding:选取区域完全属于地面；
//  * Selecting_MultipleObjectsTest_CompleteMoreTrees:选取区域完全属于多棵树；
//  * Selecting_CullingTest_KeepATree:选取区域大部分属于一棵树；
//  * Selecting_CullingTest_KeepGround:选取区域大部分属于地面；
//  * Selecting_CullingTest_KeepABuilding:选取区域大部分属于一栋建筑；
//  * Selecting_CullingTest_KeepMoreTrees:选择区域大部分属于多棵树；
//SelectingSpecialTest特定情况下的特殊结果正确


using namespace hiveObliquePhotography::PointCloudRetouch;

std::string FilePaths[][3] =
{
	{
		TESTMODEL_DIR + std::string("Test017_Model/CompleteTree/CompleteTreeInput.txt"),
		TESTMODEL_DIR + std::string("Test017_Model/CompleteTree/CompleteTreeCameraInfo.txt"),
		TESTMODEL_DIR + std::string("Test017_Model/CompleteTree/CompleteTreeGT.txt"),
	},

	{
		TESTMODEL_DIR + std::string("Test017_Model/CompleteGround/CompleteGroundInput.txt"),
		TESTMODEL_DIR + std::string("Test017_Model/CompleteGround/CompleteGroundCameraInfo.txt"),
		TESTMODEL_DIR + std::string("Test017_Model/CompleteGround/CompleteGroundGT.txt"),
	},

	{
		TESTMODEL_DIR + std::string("Test017_Model/CompleteBuilding/CompleteBuildingInput.txt"),
		TESTMODEL_DIR + std::string("Test017_Model/CompleteBuilding/CompleteBuildingCameraInfo.txt"),
		TESTMODEL_DIR + std::string("Test017_Model/CompleteBuilding/CompleteBuildingGt.txt"),
	},

	{
		TESTMODEL_DIR + std::string("Test017_Model/CompleteMoreTrees/MoreTreesInput.txt"),
		TESTMODEL_DIR + std::string("Test017_Model/CompleteMoreTrees/MoreTreesCameraInfo.txt"),
		TESTMODEL_DIR + std::string("Test017_Model/CompleteMoreTrees/MoreTreesGt.txt"),
	},

	{
		TESTMODEL_DIR + std::string("Test017_Model/KeepTree/KeepTreeInput.txt"),
		TESTMODEL_DIR + std::string("Test017_Model/KeepTree/KeepTreeCameraInfo.txt"),
		TESTMODEL_DIR + std::string("Test017_Model/KeepTree/CompleteTreeGT.txt"),
	},

	{
		TESTMODEL_DIR + std::string("Test017_Model/KeepGround/KeepGroundInput.txt"),
		TESTMODEL_DIR + std::string("Test017_Model/KeepGround/KeepGroundCameraInfo.txt"),
		TESTMODEL_DIR + std::string("Test017_Model/KeepGround/CompleteGroundGT.txt"),
	},

	{
		TESTMODEL_DIR + std::string("Test017_Model/KeepBuilding/KeepBuildingInput.txt"),
		TESTMODEL_DIR + std::string("Test017_Model/KeepBuilding/KeepBuildingCameraInfo.txt"),
		TESTMODEL_DIR + std::string("Test017_Model/KeepBuilding/CompleteBuildingGt.txt"),
	},

	{
		TESTMODEL_DIR + std::string("Test017_Model/KeepMoreTrees/KeepMoreTreesInput.txt"),
		TESTMODEL_DIR + std::string("Test017_Model/KeepMoreTrees/KeepMoreTreesCameraInfo.txt"),
		TESTMODEL_DIR + std::string("Test017_Model/KeepMoreTrees/MoreTreesGt.txt"),
	}

};

 const std::string ConfigPath = TESTMODEL_DIR + std::string("Config/Test017_PointCloudRetouchConfig.xml");

class TestSelecting : public testing::Test
{
protected:
	hiveConfig::CHiveConfig* pConfig = nullptr;
	CPointCloudRetouchManager* pManager = nullptr;
	
	void SetUp() override
	{
		pConfig = new CPointCloudRetouchConfig;
		if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, pConfig) != hiveConfig::EParseResult::SUCCEED)
		{
			_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
			return;
		}
		
		std::string ModelPath(TESTMODEL_DIR + std::string("General/slice 16.pcd"));
		PointCloud_t::Ptr pCloud(new PointCloud_t);
		pcl::io::loadPCDFile(ModelPath, *pCloud);
		pManager = CPointCloudRetouchManager::getInstance();
		pManager->init(pCloud, pConfig);
	}

	void TearDown() override
	{
		delete pConfig;
	}

	void initTest(pcl::Indices& voInputIndices, pcl::visualization::Camera& voCamera, pcl::Indices& voGroundTruth, const std::string(vPath)[3])
	{
		_loadIndices(vPath[0], voInputIndices);
		_loadCamera(vPath[1], voCamera);
		_loadIndices(vPath[2], voGroundTruth);
	}

	void loadAddtionFile(const std::string& vPath, pcl::Indices& voIndices)
	{
		_loadIndices(vPath, voIndices);
	}

private:
	void _loadIndices(const std::string& vPath, pcl::Indices& voIndices);
	void _loadCamera(const std::string& vPath, pcl::visualization::Camera& voCamera);
};

//*****************************************************************
//FUNCTION: 
void TestSelecting::_loadIndices(const std::string& vPath, pcl::Indices& voIndices)
{
	std::ifstream File(vPath);
	boost::archive::text_iarchive ia(File);
	ia >> BOOST_SERIALIZATION_NVP(voIndices);
	File.close();
}

//*****************************************************************
//FUNCTION: 
void TestSelecting::_loadCamera(const std::string& vPath, pcl::visualization::Camera& voCamera)
{
	auto pVisualizer = new pcl::visualization::PCLVisualizer("Viewer", true);
	pVisualizer->loadCameraParameters(vPath);
	pVisualizer->getCameraParameters(voCamera);
}

double distanceFunc(Eigen::Vector2d vInput)
{
	return -1;
}

TEST_F(TestSelecting, Selecting_NoThroughTest_CompleteTree)
{
	const auto& Path = FilePaths[0];

	pcl::Indices InputIndices;
	pcl::visualization::Camera Camera;
	pcl::Indices GroundTruth;

	initTest(InputIndices, Camera, GroundTruth, Path);

	Eigen::Matrix4d ViewMatrix, ProjectionMatrix;
	Camera.computeViewMatrix(ViewMatrix);
	Camera.computeProjectionMatrix(ProjectionMatrix);
	const Eigen::Matrix4d PvMatrix = ProjectionMatrix * ViewMatrix;
	CPointSetPreprocessor CullingOperation;
	CullingOperation.cullBySdf(InputIndices, PvMatrix, distanceFunc);
	Eigen::Vector3d ViewPos(Camera.pos[0], Camera.pos[1], Camera.pos[2]);
	CullingOperation.cullByDepth(InputIndices, PvMatrix, ViewPos);
	
	pcl::Indices Difference;
	std::set_difference(InputIndices.begin(), InputIndices.end(),
		GroundTruth.begin(), GroundTruth.end(),
		std::inserter(Difference, Difference.begin()));

	GTEST_ASSERT_LE(Difference.size(), 0);
}

TEST_F(TestSelecting, Selecting_NoThroughTest_CompleteGround)
{
	const auto& Path = FilePaths[1];

	pcl::Indices InputIndices;
	pcl::visualization::Camera Camera;
	pcl::Indices GroundTruth;

	initTest(InputIndices, Camera, GroundTruth, Path);

	Eigen::Matrix4d ViewMatrix, ProjectionMatrix;
	Camera.computeViewMatrix(ViewMatrix);
	Camera.computeProjectionMatrix(ProjectionMatrix);
	const Eigen::Matrix4d PvMatrix = ProjectionMatrix * ViewMatrix;
	CPointSetPreprocessor CullingOperation;
	CullingOperation.cullBySdf(InputIndices, PvMatrix, distanceFunc);
	Eigen::Vector3d ViewPos(Camera.pos[0], Camera.pos[1], Camera.pos[2]);
	CullingOperation.cullByDepth(InputIndices, PvMatrix, ViewPos);

	pcl::Indices Difference;
	std::set_difference(InputIndices.begin(), InputIndices.end(),
		GroundTruth.begin(), GroundTruth.end(),
		std::inserter(Difference, Difference.begin()));

	GTEST_ASSERT_LE(Difference.size(), 0);
}

TEST_F(TestSelecting, Selecting_NoThroughTest_CompleteBuilding)
{
	std::string ModelPath(TESTMODEL_DIR + std::string("General/slice 15.pcd"));
	PointCloud_t::Ptr pTempCloud(new PointCloud_t);
	pcl::io::loadPCDFile(ModelPath, *pTempCloud);
	pManager = CPointCloudRetouchManager::getInstance();
	pManager->init(pTempCloud, pConfig);

	const auto& Path = FilePaths[2];

	pcl::Indices InputIndices;
	pcl::visualization::Camera Camera;
	pcl::Indices GroundTruth;

	initTest(InputIndices, Camera, GroundTruth, Path);

	Eigen::Matrix4d ViewMatrix, ProjectionMatrix;
	Camera.computeViewMatrix(ViewMatrix);
	Camera.computeProjectionMatrix(ProjectionMatrix);
	const Eigen::Matrix4d PvMatrix = ProjectionMatrix * ViewMatrix;
	CPointSetPreprocessor CullingOperation;
	CullingOperation.cullBySdf(InputIndices, PvMatrix, distanceFunc);
	Eigen::Vector3d ViewPos(Camera.pos[0], Camera.pos[1], Camera.pos[2]);
	CullingOperation.cullByDepth(InputIndices, PvMatrix, ViewPos);

	pcl::Indices Difference;
	std::set_difference(InputIndices.begin(), InputIndices.end(),
		GroundTruth.begin(), GroundTruth.end(),
		std::inserter(Difference, Difference.begin()));

	GTEST_ASSERT_LE(Difference.size(), 1);
}

TEST_F(TestSelecting, Selecting_MultipleObjectsTest_CompleteMoreTrees)
{
	const auto& Path = FilePaths[3];

	pcl::Indices InputIndices;
	pcl::visualization::Camera Camera;
	pcl::Indices GroundTruth;

	initTest(InputIndices, Camera, GroundTruth, Path);

	Eigen::Matrix4d ViewMatrix, ProjectionMatrix;
	Camera.computeViewMatrix(ViewMatrix);
	Camera.computeProjectionMatrix(ProjectionMatrix);
	const Eigen::Matrix4d PvMatrix = ProjectionMatrix * ViewMatrix;
	CPointSetPreprocessor CullingOperation;
	CullingOperation.cullBySdf(InputIndices, PvMatrix, distanceFunc);
	Eigen::Vector3d ViewPos(Camera.pos[0], Camera.pos[1], Camera.pos[2]);
	CullingOperation.cullByDepth(InputIndices, PvMatrix, ViewPos);

	pcl::Indices Difference;
	std::set_difference(InputIndices.begin(), InputIndices.end(),
		GroundTruth.begin(), GroundTruth.end(),
		std::inserter(Difference, Difference.begin()));
	GTEST_ASSERT_LE(Difference.size(), 0);

	pcl::Indices AdditionIndices;
	loadAddtionFile(TESTMODEL_DIR + std::string("Test017_Model/CompleteMoreTrees/MoreTreesGt2.txt"), AdditionIndices);
	pcl::Indices Interaction;
	std::set_intersection(InputIndices.begin(), InputIndices.end(),
		AdditionIndices.begin(), AdditionIndices.end(),
		std::inserter(Interaction, Interaction.begin()));
	GTEST_ASSERT_GE(Interaction.size(), 1);

	pcl::Indices OtherAdditionIndices;
	loadAddtionFile(TESTMODEL_DIR + std::string("Test017_Model/CompleteMoreTrees/MoreTreesGt3.txt"), OtherAdditionIndices);
	pcl::Indices OtherInteraction;
	std::set_intersection(InputIndices.begin(), InputIndices.end(),
		OtherAdditionIndices.begin(), OtherAdditionIndices.end(),
		std::inserter(OtherInteraction, OtherInteraction.begin()));
	GTEST_ASSERT_GE(OtherInteraction.size(), 1);
}

TEST_F(TestSelecting, Selecting_CullingTest_KeepATree)
{
	const auto& Path = FilePaths[4];

	pcl::Indices InputIndices;
	pcl::visualization::Camera Camera;
	pcl::Indices GroundTruth;

	initTest(InputIndices, Camera, GroundTruth, Path);

	Eigen::Matrix4d ViewMatrix, ProjectionMatrix;
	Camera.computeViewMatrix(ViewMatrix);
	Camera.computeProjectionMatrix(ProjectionMatrix);
	const Eigen::Matrix4d PvMatrix = ProjectionMatrix * ViewMatrix;
	CPointSetPreprocessor CullingOperation;
	CullingOperation.cullBySdf(InputIndices, PvMatrix, distanceFunc);
	Eigen::Vector3d ViewPos(Camera.pos[0], Camera.pos[1], Camera.pos[2]);
	CullingOperation.cullByDepth(InputIndices, PvMatrix, ViewPos);

	pcl::Indices Difference;
	std::set_difference(InputIndices.begin(), InputIndices.end(),
		GroundTruth.begin(), GroundTruth.end(),
		std::inserter(Difference, Difference.begin()));

	GTEST_ASSERT_LE(Difference.size(), 0);
}

TEST_F(TestSelecting, Selecting_CullingTest_KeepGround)
{
	const auto& Path = FilePaths[5];

	pcl::Indices InputIndices;
	pcl::visualization::Camera Camera;
	pcl::Indices GroundTruth;

	initTest(InputIndices, Camera, GroundTruth, Path);

	Eigen::Matrix4d ViewMatrix, ProjectionMatrix;
	Camera.computeViewMatrix(ViewMatrix);
	Camera.computeProjectionMatrix(ProjectionMatrix);
	const Eigen::Matrix4d PvMatrix = ProjectionMatrix * ViewMatrix;
	CPointSetPreprocessor CullingOperation;
	CullingOperation.cullBySdf(InputIndices, PvMatrix, distanceFunc);
	Eigen::Vector3d ViewPos(Camera.pos[0], Camera.pos[1], Camera.pos[2]);
	CullingOperation.cullByDepth(InputIndices, PvMatrix, ViewPos);

	pcl::Indices Difference;
	std::set_difference(InputIndices.begin(), InputIndices.end(),
		GroundTruth.begin(), GroundTruth.end(),
		std::inserter(Difference, Difference.begin()));

	GTEST_ASSERT_LE(Difference.size(), 0);
}

TEST_F(TestSelecting, Selecting_CullingTest_KeepABuilding)
{
	std::string ModelPath(TESTMODEL_DIR + std::string("General/slice 15.pcd"));
	PointCloud_t::Ptr pTempCloud(new PointCloud_t);
	pcl::io::loadPCDFile(ModelPath, *pTempCloud);
	pManager = CPointCloudRetouchManager::getInstance();
	pManager->init(pTempCloud, pConfig);

	const auto& Path = FilePaths[6];

	pcl::Indices InputIndices;
	pcl::visualization::Camera Camera;
	pcl::Indices GroundTruth;

	initTest(InputIndices, Camera, GroundTruth, Path);

	Eigen::Matrix4d ViewMatrix, ProjectionMatrix;
	Camera.computeViewMatrix(ViewMatrix);
	Camera.computeProjectionMatrix(ProjectionMatrix);
	const Eigen::Matrix4d PvMatrix = ProjectionMatrix * ViewMatrix;
	CPointSetPreprocessor CullingOperation;
	CullingOperation.cullBySdf(InputIndices, PvMatrix, distanceFunc);
	Eigen::Vector3d ViewPos(Camera.pos[0], Camera.pos[1], Camera.pos[2]);
	CullingOperation.cullByDepth(InputIndices, PvMatrix, ViewPos);

	pcl::Indices Difference;
	std::set_difference(InputIndices.begin(), InputIndices.end(),
		GroundTruth.begin(), GroundTruth.end(),
		std::inserter(Difference, Difference.begin()));

	GTEST_ASSERT_LE(Difference.size(), 0);
}

TEST_F(TestSelecting, Selecting_CullingTest_KeepMoreTrees)
{
	const auto& Path = FilePaths[7];

	pcl::Indices InputIndices;
	pcl::visualization::Camera Camera;
	pcl::Indices GroundTruth;

	initTest(InputIndices, Camera, GroundTruth, Path);

	Eigen::Matrix4d ViewMatrix, ProjectionMatrix;
	Camera.computeViewMatrix(ViewMatrix);
	Camera.computeProjectionMatrix(ProjectionMatrix);
	const Eigen::Matrix4d PvMatrix = ProjectionMatrix * ViewMatrix;
	CPointSetPreprocessor CullingOperation;
	CullingOperation.cullBySdf(InputIndices, PvMatrix, distanceFunc);
	Eigen::Vector3d ViewPos(Camera.pos[0], Camera.pos[1], Camera.pos[2]);
	CullingOperation.cullByDepth(InputIndices, PvMatrix, ViewPos);

	pcl::Indices Difference;
	std::set_difference(InputIndices.begin(), InputIndices.end(),
		GroundTruth.begin(), GroundTruth.end(),
		std::inserter(Difference, Difference.begin()));

	GTEST_ASSERT_LE(Difference.size(), 0);
}
