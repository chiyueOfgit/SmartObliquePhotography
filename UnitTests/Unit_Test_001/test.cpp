#include "pch.h"
#include "../../ObliquePhotographyData/PointCloudPLYLoader.h"

//测试用例列表：
//  * LoadTile: 能够正常的载入一个格式为？？的点云文件
//  * DeathTest_LoadInexistentTile：尝试载入一个不存在的点云文件
//  * DeathTest_LoadUnsupportedFormat: 尝试载入一个不支持格式的点云文件

const std::string g_ValidFileName;
const std::string g_InvalidFileName;

TEST(Test_LoadPointCloudTile, LoadTile) 
{
	hiveObliquePhotography::CPointCloudPLYLoader Loader;
	Loader.loadDataFromFile(g_ValidFileName);
}

TEST(Test_LoadPointCloudTile, DeathTest_LoadInexistentTile)
{
}
