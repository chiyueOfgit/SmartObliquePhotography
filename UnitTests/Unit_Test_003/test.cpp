#include "pch.h"

#include <common/FileSystem.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <common/MathInterface.h>

#include "PointCloudScene.h"

using namespace hiveObliquePhotography;

//测试用例列表：
//  * SaveScene4PLY: 能够正常的保存(覆盖)为PLY点云文件,且数据正确
//  * SaveScene4PCD：能够正常的保存(覆盖)为PCD点云文件,且数据正确
//  * Save4NonexistentPath: 能够保存场景为不存在的文件名

const std::string g_InputPath = TESTMODEL_DIR + std::string("General/slice 1.pcd");
const std::string g_ValidPLYFilePath = TESTMODEL_DIR + std::string("Test003_Model/slice 1.ply");
const std::string g_ValidPCDFilePath = TESTMODEL_DIR + std::string("Test003_Model/slice 1.pcd");
const std::string g_NonexistentFormatFileName = TESTMODEL_DIR + std::string("Test003_Model/slice 2.ply");

TEST(Test_SavePointCloudScene, SaveScene4PLY)
{
	PointCloud_t::Ptr pCloud(new PointCloud_t);
	pcl::io::loadPCDFile(g_InputPath, *pCloud);
	int InputSize = pCloud->size();
	auto RandomIndex = hiveMath::hiveGenerateRandomInteger(0, InputSize);
	auto RandomPoint = pCloud->points[RandomIndex];
	Eigen::Vector3f RandomPointPos= RandomPoint.getVector3fMap();
	
	EXPECT_NO_THROW(CPointCloudScene::getInstance()->saveScene(*pCloud, g_ValidPLYFilePath));
	
	PointCloud_t::Ptr pSavedCloud(new PointCloud_t);
	pcl::io::loadPLYFile(g_ValidPLYFilePath, *pSavedCloud);
	int SavedSize = pSavedCloud->size();
	ASSERT_NE(SavedSize, 0);
	auto RandomSavedPoint = pSavedCloud->points[RandomIndex];
	Eigen::Vector3f RandomSavedPointPos = RandomSavedPoint.getVector3fMap();
	
	EXPECT_EQ(InputSize, SavedSize);
	EXPECT_EQ(RandomPointPos, RandomSavedPointPos);
}

TEST(Test_SavePointCloudScene, SaveScene4PCD)
{
	PointCloud_t::Ptr pCloud(new PointCloud_t);
	pcl::io::loadPCDFile(g_InputPath, *pCloud);
	int InputSize = pCloud->size();
	auto RandomIdex = hiveMath::hiveGenerateRandomInteger(0, InputSize);
	auto RandomPoint = pCloud->points[RandomIdex];
	Eigen::Vector3f RandomPointPos = RandomPoint.getVector3fMap();

	EXPECT_NO_THROW(CPointCloudScene::getInstance()->saveScene(*pCloud, g_ValidPCDFilePath));

	PointCloud_t::Ptr pSavedCloud(new PointCloud_t);
	pcl::io::loadPCDFile(g_ValidPCDFilePath, *pSavedCloud);
	int SavedSize = pSavedCloud->size();
	ASSERT_NE(SavedSize, 0);
	auto RandomSavedPoint = pSavedCloud->points[RandomIdex];
	Eigen::Vector3f RandomSavedPointPos = RandomSavedPoint.getVector3fMap();

	EXPECT_EQ(InputSize, SavedSize);
	EXPECT_EQ(RandomPointPos, RandomSavedPointPos);
}

TEST(Test_SavePointCloudScene, Save4NonexistentFormat)
{
	hiveUtility::hiveFileSystem::hiveRemoveFile(g_NonexistentFormatFileName);
	
	PointCloud_t::Ptr pCloud(new PointCloud_t);
	pcl::io::loadPCDFile(g_InputPath, *pCloud);
	int InputSize = pCloud->size();
	auto RandomIdex = hiveMath::hiveGenerateRandomInteger(0, InputSize);
	auto RandomPoint = pCloud->points[RandomIdex];

	EXPECT_NO_THROW(CPointCloudScene::getInstance()->saveScene(*pCloud, g_NonexistentFormatFileName));

	PointCloud_t::Ptr pSavedCloud(new PointCloud_t);
	pcl::io::loadPCDFile(g_ValidPCDFilePath, *pSavedCloud);
	int SavedSize = pSavedCloud->size();
	ASSERT_NE(SavedSize, 0);
	
	EXPECT_EQ(InputSize, SavedSize);
}
