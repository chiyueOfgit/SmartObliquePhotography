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
		"../TestModel/General/slice 1.ply", //148701
		"../TestModel/General/slice 2.pcd", //225016
		"../TestModel/General/slice 3.pcd", //227563
		"../TestModel/General/slice 4.pcd"  //225220
	};
	auto PointCloud = CPointCloudScene::getInstance()->loadScene(FilePaths);
	auto num = CPointCloudScene::getInstance()->getNumTiles();
	GTEST_ASSERT_EQ(num, 4);
	GTEST_ASSERT_EQ(PointCloud->size(), 826500);
}

TEST(Test_LoadPointCloudScene, DeathTest_LoadDuplicatedFile1)
{
	const std::vector<std::string> FilePaths{
		"../TestModel/General/slice 1.pcd",
		"../TestModel/General/slice 2.pcd",
		"../TestModel/General/slice 2.pcd",
		"../TestModel/General/slice 3.pcd"
	};
	
	auto PointCloud = CPointCloudScene::getInstance()->loadScene(FilePaths);
	
	GTEST_ASSERT_EQ(CPointCloudScene::getInstance()->getNumTiles(), 3);
	GTEST_ASSERT_EQ(PointCloud->size(), 601280);
}

TEST(Test_LoadPointCloudScene, DeathTest_LoadDuplicatedFile2)
{
	const std::vector<std::string> FilePaths{
		"../TestModel/General/slice 1.pcd",
		"../TestModel/General/Slice 2.pcd",
		"../TestModel/General/slice 2.pcd",
		"../TestModel/General/slice 3.pcd"
	};

	auto PointCloud = CPointCloudScene::getInstance()->loadScene(FilePaths);

	GTEST_ASSERT_EQ(CPointCloudScene::getInstance()->getNumTiles(), 3);
	GTEST_ASSERT_EQ(PointCloud->size(), 601280);
}

TEST(Test_LoadPointCloudScene, DeathTest_LoadPartiallyIncorrectFileSet)
{
	std::vector<std::string> FilePaths{
		"../TestModel/General/slice 1.pcd",
		"../TestModel/General/slice 2.pcd",
		"../TestModel/General/slice 3.pcd",
		"../TestModel/General/slice 5.pcd",
		"../TestModel/General/slice 1.txt",
	};
	auto PointCloud = CPointCloudScene::getInstance()->loadScene(FilePaths);
	
	GTEST_ASSERT_EQ(CPointCloudScene::getInstance()->getNumTiles(), 3);
	GTEST_ASSERT_EQ(PointCloud->size(), 601280);
}

TEST(Test_LoadPointCloudScene, DeathTest_LoadIncorrectFileSet)
{
	std::vector<std::string> FilePaths{ "slice 1.pcd" };

	auto PointCloud = CPointCloudScene::getInstance()->loadScene(FilePaths);

	GTEST_ASSERT_EQ(CPointCloudScene::getInstance()->getNumTiles(), 1);
	GTEST_ASSERT_EQ(PointCloud->size(), 1);
}