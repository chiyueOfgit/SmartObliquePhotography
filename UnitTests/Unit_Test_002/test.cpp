#include "pch.h"
#include "PointCloudScene.h"

using namespace hiveObliquePhotography;

//测试用例列表：
//  * LoadScene: 能够正常的载入一堆点云文件
//  * DeathTest_LoadDuplicatedFile：载入的点云文件中有重复
//  * DeathTest_LoadPartiallyIncorrectFileSet: 载入的点云文件中，有部分文件错误（不存在，格式不支持...）


TEST(Test_LoadPointCloudScene, LoadScene)
{
	const std::vector<std::string> FilePaths{
		"TestModel/slice 1.ply", //148701
		"TestModel/slice 2.pcd", //225016
		"TestModel/slice 3.pcd", //227563
		"TestModel/slice 4.pcd"  //225220
	};
	auto PointCloud = CPointCloudScene::getInstance()->loadScene(FilePaths);
	
	GTEST_ASSERT_EQ(CPointCloudScene::getInstance()->getNumTiles(), 4);
	GTEST_ASSERT_EQ(PointCloud->size(), 826500);

}

TEST(Test_LoadPointCloudScene, DeathTest_LoadDuplicatedFile)
{
	const std::vector<std::string> FilePaths{
		"TestModel/slice 1.pcd",
		"TestModel/slice 2.pcd",
		"TestModel/slice 2.pcd",
		"TestModel/slice 3.pcd"
	};
	
	auto PointCloud = CPointCloudScene::getInstance()->loadScene(FilePaths);
	
	GTEST_ASSERT_EQ(CPointCloudScene::getInstance()->getNumTiles(), 3);
	GTEST_ASSERT_EQ(PointCloud->size(), 601280);
}

TEST(Test_LoadPointCloudScene, DeathTest_LoadPartiallyIncorrectFileSet)
{
	std::vector<std::string> FilePaths{
		"TestModel/slice 1.pcd",
		"TestModel/slice 2.pcd",
		"TestModel/slice 3.pcd",
		"TestModel/slice 5.pcd",
		"TestModel/slice 1.txt",
	};
	auto PointCloud = CPointCloudScene::getInstance()->loadScene(FilePaths);
	
	GTEST_ASSERT_EQ(CPointCloudScene::getInstance()->getNumTiles(), 3);
	GTEST_ASSERT_EQ(PointCloud->size(), 601280);
}