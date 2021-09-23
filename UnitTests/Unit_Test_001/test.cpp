#include "pch.h"
#include <common/UtilityInterface.h>
#include "PointCloudLoader.h"

using namespace hiveObliquePhotography;

//测试用例列表：
//  * LoadTile: 能够正常的载入一个格式为？？的点云文件
//  * DeathTest_LoadInexistentTile：尝试载入一个不存在的点云文件
//  * DeathTest_LoadUnsupportedFormat: 尝试载入一个不支持格式的点云文件

const std::string g_ValidPLYFileName = TESTMODEL_DIR + std::string("General/slice 1.ply");
const std::string g_ValidPCDFileName = TESTMODEL_DIR + std::string("General/slice 1.pcd");
const std::string g_InexistentFileName = TESTMODEL_DIR + std::string("General/slice 5.pcd");
const std::string g_UnsupportedFileName = TESTMODEL_DIR + std::string("General/slice 1.txt");

TEST(Test_LoadPointCloudTile, LoadTilePly)
{	
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<IPointCloudLoader>(hiveUtility::hiveGetFileSuffix(g_ValidPLYFileName));
	auto pTile = pTileLoader->loadDataFromFile(g_ValidPLYFileName);
	ASSERT_NE(pTile, nullptr);
	GTEST_ASSERT_EQ(pTile->size(), 148701);
}

TEST(Test_LoadPointCloudTile, LoadTilePcd)
{
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<IPointCloudLoader>(hiveUtility::hiveGetFileSuffix(g_ValidPCDFileName));
	auto pTile = pTileLoader->loadDataFromFile(g_ValidPCDFileName);
	ASSERT_NE(pTile, nullptr);
	GTEST_ASSERT_EQ(pTile->size(), 148701);
}

TEST(Test_LoadPointCloudTile, DeathTest_LoadInexistentTile)
{
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<IPointCloudLoader>(hiveUtility::hiveGetFileSuffix(g_InexistentFileName));
	std::shared_ptr<PointCloud_t> pTile;
	EXPECT_ANY_THROW(pTile = pTileLoader->loadDataFromFile(g_InexistentFileName));
	EXPECT_EQ(pTile, nullptr);
}
