#include "pch.h"

#include <common/MathInterface.h>
#include "PointCloudRetouchInterface.h"
#include "PointCloudRetouchConfig.h"
#include <fstream>
#include <boost/archive/text_oarchive.hpp> 
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <pcl/io/pcd_io.h>

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

std::string FilePaths[][3] =
{
	{
		"../TestModel/Test017_Model/CompleteTree/CompleteTreeInput.txt",
		"../TestModel/Test017_Model/CompleteTree/CompleteTreeCameraInfo.txt",
		"../TestModel/Test017_Model/CompleteTree/CompleteTreeGT.txt",
	},

	{
		"../TestModel/Test017_Model/CompleteGround/CompleteGroundInput.txt",
		"../TestModel/Test017_Model/CompleteGround/CompleteGroundCameraInfo.txt",
		"../TestModel/Test017_Model/CompleteGround/CompleteGroundGT.txt",
	},

	{
		"../TestModel/Test017_Model/CompleteBuilding/CompleteBuildingInput.txt",
		"../TestModel/Test017_Model/CompleteBuilding/CompleteBuildingCameraInfo.txt",
		"../TestModel/Test017_Model/CompleteBuilding/CompleteBuildingGt.txt",
	},

	{
		"../TestModel/Test017_Model/CompleteMoreTrees/MoreTreesInput.txt",
		"../TestModel/Test017_Model/CompleteMoreTrees/MoreTreesCameraInfo.txt",
		"../TestModel/Test017_Model/CompleteMoreTrees/MoreTreesGt.txt",
	},
	
	{
		"../TestModel/Test017_Model/KeepTree/KeepTreeInput.txt",
		"../TestModel/Test017_Model/KeepTree/KeepTreeCameraInfo.txt",
		"../TestModel/Test017_Model/KeepTree/CompleteTreeGT.txt",
	},
	
	{
		"../TestModel/Test017_Model/KeepGround/KeepGroundInput.txt",
		"../TestModel/Test017_Model/KeepGround/KeepGroundCameraInfo.txt",
		"../TestModel/Test017_Model/KeepGround/CompleteGroundGT.txt",
	},
	
	{
		"../TestModel/Test017_Model/KeepBuilding/KeepBuildingInput.txt",
		"../TestModel/Test017_Model/KeepBuilding/KeepBuildingCameraInfo.txt",
		"../TestModel/Test017_Model/KeepBuilding/CompleteBuildingGt.txt",
	},
	
	{
		"../TestModel/Test017_Model/KeepMoreTrees/KeepMoreTreesInput.txt",
		"../TestModel/Test017_Model/KeepMoreTrees/KeepMoreTreesCameraInfo.txt",
		"../TestModel/Test017_Model/KeepMoreTrees/MoreTreesGt.txt",
	}
	
};


using namespace hiveObliquePhotography::PointCloudRetouch;

class TestSelecting : public testing::Test
{
protected:
	void SetUp() override
	{
		std::string ModelPath("../TestModel/General/slice 16.pcd");
		PointCloud_t::Ptr pCloud(new PointCloud_t);
		pcl::io::loadPCDFile(ModelPath, *pCloud);
		
	}

	void TearDown() override
	{
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

TEST_F(TestSelecting, Selecting_NoThroughTest_CompleteTree)
{
	const auto& Path = FilePaths[0];

	pcl::Indices InputIndices;
	pcl::visualization::Camera Camera;
	pcl::Indices GroundTruth;

	initTest(InputIndices, Camera, GroundTruth, Path);

	//TODO:根据接口执行选择剔除
	
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

	//TODO:根据接口执行选择剔除

	pcl::Indices Difference;
	std::set_difference(InputIndices.begin(), InputIndices.end(),
		GroundTruth.begin(), GroundTruth.end(),
		std::inserter(Difference, Difference.begin()));

	GTEST_ASSERT_LE(Difference.size(), 0);
}

TEST_F(TestSelecting, Selecting_NoThroughTest_CompleteBuilding)
{
	const auto& Path = FilePaths[2];

	pcl::Indices InputIndices;
	pcl::visualization::Camera Camera;
	pcl::Indices GroundTruth;

	initTest(InputIndices, Camera, GroundTruth, Path);

	//TODO:根据接口执行选择剔除

	pcl::Indices Difference;
	std::set_difference(InputIndices.begin(), InputIndices.end(),
		GroundTruth.begin(), GroundTruth.end(),
		std::inserter(Difference, Difference.begin()));

	GTEST_ASSERT_LE(Difference.size(), 0);
}

TEST_F(TestSelecting, Selecting_MultipleObjectsTest_CompleteMoreTrees)
{
	const auto& Path = FilePaths[3];

	pcl::Indices InputIndices;
	pcl::visualization::Camera Camera;
	pcl::Indices GroundTruth;

	initTest(InputIndices, Camera, GroundTruth, Path);

	//TODO:根据接口执行选择剔除

	pcl::Indices Difference;
	std::set_difference(InputIndices.begin(), InputIndices.end(),
		GroundTruth.begin(), GroundTruth.end(),
		std::inserter(Difference, Difference.begin()));
	GTEST_ASSERT_LE(Difference.size(), 0);
	
	pcl::Indices AdditionIndices;
	loadAddtionFile("../TestModel/Test017_Model/CompleteMoreTrees/MoreTreesGt2.txt", AdditionIndices);
	pcl::Indices Interaction;
	std::set_intersection(InputIndices.begin(), InputIndices.end(),
		AdditionIndices.begin(), AdditionIndices.end(),
		std::inserter(Interaction, Interaction.begin()));
	GTEST_ASSERT_GE(Interaction.size(), 1);

	pcl::Indices OtherAdditionIndices;
	loadAddtionFile("../TestModel/Test017_Model/CompleteMoreTrees/MoreTreesGt3.txt", OtherAdditionIndices);
	pcl::Indices OtherInteraction;
	std::set_intersection(InputIndices.begin(), InputIndices.end(),
		OtherAdditionIndices.begin(), OtherAdditionIndices.end(),
		std::inserter(OtherInteraction, OtherInteraction.begin()));
	GTEST_ASSERT_GE(OtherInteraction.size(), 1);
}

