#include "pch.h"
#include "PointCloudRetouchInterface.h"
#include "PointCloudRetouchScene.h"

//主要围绕CPointCloudRetouchManager::init()进行测试
//注意不要直接一来就直接测试这个函数，看看这个函数的实现，目前在其内部调用了其他几个类的公有函数，要分别针对这些公有函数
//进行测试，最后才是测试CPointCloudRetouchManager::init()

//测试用例列表：
//  * InitPointCloudRetouchScene: 能够正常初始化PointCloudRetouchScene
//  * DeathTest_InitSceneWithErrorptr: 尝试用错指针初始化Scene
//  * DeathTest_InitSceneWithErrorPtr: 尝试用未初始化的乱指针初始化Scene
//  * InitPointLabelSet: 能够正常初始化PointLabelSet
//  * 
//  * DeathTest_LoadInexistentTile：尝试载入一个不存在的点云文件
//  * DeathTest_LoadUnsupportedFormat: 尝试载入一个不支持格式的点云文件

using namespace hiveObliquePhotography::PointCloudRetouch;

const std::string g_CloudPath = "Panda.pcd";

TEST(Test_InitPointCloudRetouch, InitPointCloudRetouchScene)
{
	PointCloud_t::Ptr pCloud(new PointCloud_t);
	pcl::io::loadPCDFile(g_CloudPath, *pCloud);

	CPointCloudRetouchScene Scene;
	Scene.init(pCloud);

	ASSERT_EQ(Scene.getNumPoint(), 16145);
	ASSERT_EQ(Scene.getPointCloudScene(), pCloud);
	ASSERT_EQ(Scene.getGlobalKdTree()->getInputCloud(), Scene.getPointCloudScene());
}

TEST(Test_InitPointCloudRetouch, DeathTest_InitSceneWithErrorPtr)
{
	//空指针
	{
		CPointCloudRetouchScene Scene;
		ASSERT_ANY_THROW(Scene.init(nullptr));
	}

	//未定义的乱指针
	{
		CPointCloudRetouchScene Scene;
		PointCloud_t::Ptr pCloud;
		ASSERT_ANY_THROW(Scene.init(pCloud));
	}

	//
	{
		CPointCloudRetouchScene Scene;
		PointCloud_t::ConstPtr pCloud(new PointCloud_t);
		pcl::io::loadPCDFile(g_CloudPath, *pCloud);
		Scene.init(pCloud);
	}

}


