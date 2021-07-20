#include "pch.h"
#include "PointCloudScene.h"

using namespace hiveObliquePhotography;

//测试用例列表：
//  * SaveScene4PLY: 能够正常的保存为PLY点云文件
//  * SaveScene4PLY：能够正常的保存为PCD点云文件
//  * Save4NonexistentPath: 保存场景为不存在的文件名

const std::string g_ValidPLYFilePath = TESTMODEL_DIR + std::string("Test003_Model/slice 1.ply");
const std::string g_ValidPCDFilePath = TESTMODEL_DIR + std::string("Test003_Model/slice 1.pcd");
const std::string g_NonexistentFormatFileName = TESTMODEL_DIR + std::string("Test003_Model/slice 2.ply");

TEST(Test_SavePointCloudScene, SaveScene4PLY)
{
	
}

TEST(Test_SavePointCloudScene, SaveScene4PCD)
{
	
}

TEST(Test_SavePointCloudScene, DeathTest_Save4NonexistentFormat)
{
	
}
