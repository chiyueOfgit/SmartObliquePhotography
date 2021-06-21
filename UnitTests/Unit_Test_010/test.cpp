#include "pch.h"
#include "PointCloudRetouchInterface.h"
#include "PointCloudRetouchScene.h"
#include "PointLabelSet.h"
#include "NeighborhoodBuilder.h"
#include "RetouchTask.h"
#include "PointCloudRetouchManager.h"
#include "PointCloudRetouchConfig.h"

//主要围绕CPointCloudRetouchManager::init()进行测试
//注意不要直接一来就直接测试这个函数，看看这个函数的实现，目前在其内部调用了其他几个类的公有函数，要分别针对这些公有函数
//进行测试，最后才是测试CPointCloudRetouchManager::init()

//测试用例列表：
//  * InitPointCloudRetouchScene: 能够正常初始化PointCloudRetouchScene
//  * DeathTest_InitSceneWithErrorptr: 尝试用错指针初始化Scene
// 
//  * InitPointLabelSet: 能够正常初始化PointLabelSet
//  * DeathTest_InitSceneWithNegativeSize: 尝试用负数大小初始化LabelSet
// 
//  * CreateNeighborhoodBuilder: 能够成功创建出NeighborhoodBuilder
// 
//  * InitRetouchTask: 能够正常初始化RetouchTask
//  * DeathTest_InitRetouchTaskWithErrorConfig: 尝试用错Config初始化Task
// 
//  * InitPointCloudRetouchManager: 能够正常初始化Manager

using namespace hiveObliquePhotography::PointCloudRetouch;

const std::string g_CloudPath = "../../UnitTests/Unit_Test_010/Panda.pcd";

const std::string g_BuilderSig = "";

const std::string g_ConfigPath = "Config";
const std::string g_LitterSig = "LitterMarker";
const std::string g_BackgroundSig = "BackgroundMarker";

TEST(Test_InitPointCloudRetouch, InitPointCloudRetouchScene)
{
	PointCloud_t::Ptr pCloud(new PointCloud_t);
	pcl::io::loadPCDFile(g_CloudPath, *pCloud);
	CPointCloudRetouchScene Scene;

	Scene.init(pCloud);
	ASSERT_EQ(Scene.getNumPoint(), 16145);
}

TEST(Test_InitPointCloudRetouch, DeathTest_InitSceneWithErrorPtr)
{
	//空指针
	{
		CPointCloudRetouchScene Scene;
		EXPECT_ANY_THROW(Scene.init(nullptr));
	}

	//未定义的乱指针
	{
		CPointCloudRetouchScene Scene;
		PointCloud_t::Ptr pCloud;
		EXPECT_ANY_THROW(Scene.init(pCloud));
	}
}

TEST(Test_InitPointCloudRetouch, InitPointLabelSet)
{
	CPointLabelSet LabelSet;
	const std::size_t Num = 10;

	LabelSet.init(Num);
	ASSERT_EQ(LabelSet.getSize(), Num);

	for (int i = 0; i < Num; i++)
	{
		ASSERT_EQ(LabelSet.getClusterIndexAt(i), 0);
		ASSERT_EQ(LabelSet.getLabelAt(i), EPointLabel::UNDETERMINED);
	}
}

TEST(Test_InitPointCloudRetouch, DeathTest_InitSceneWithNegativeSize)
{
	CPointLabelSet LabelSet;

	//负数要么throw要么初始化为空
	EXPECT_ANY_THROW(LabelSet.init(-1));
	EXPECT_NE(LabelSet.getSize(), -1);
	EXPECT_EQ(LabelSet.getSize(), 0);
}

TEST(Test_InitPointCloudRetouch, CreateNeighborhoodBuilder)
{
	PointCloud_t::Ptr pCloud(new PointCloud_t);
	pcl::io::loadPCDFile(g_CloudPath, *pCloud);

	INeighborhoodBuilder* pBuilder = hiveDesignPattern::hiveCreateProduct<INeighborhoodBuilder>(g_BuilderSig, pCloud, nullptr);
	ASSERT_NE(pBuilder, nullptr);
	auto pTag = pBuilder->getVisitedTag();
	ASSERT_TRUE(!pCloud->empty());
	EXPECT_NO_THROW(pTag[pCloud->size() - 1]);
	for (int i = 0; i < pCloud->size(); i++)
		ASSERT_EQ(pTag[i], false);
}

TEST(Test_InitPointCloudRetouch, InitRetouchTask)
{
	hiveConfig::CHiveConfig* pConfig = nullptr;
	hiveConfig::hiveParseConfig(g_ConfigPath, hiveConfig::EConfigType::XML, pConfig);

	for (auto i = 0; i < pConfig->getNumSubconfig(); i++)
	{
		const hiveConfig::CHiveConfig* pSubConfig = pConfig->getSubconfigAt(i);
		if (_IS_STR_IDENTICAL(pSubConfig->getName(), std::string(g_LitterSig)))
		{
			CRetouchTask LitterMarker;

			LitterMarker.init(pSubConfig);
			ASSERT_NE(LitterMarker.getExpander(), nullptr);
		}
		else if (_IS_STR_IDENTICAL(pSubConfig->getName(), std::string(g_BackgroundSig)))
		{
			CRetouchTask BackgroundMarker;

			BackgroundMarker.init(pSubConfig);
			ASSERT_NE(BackgroundMarker.getExpander(), nullptr);
		}
	}
}

TEST(Test_InitPointCloudRetouch, DeathTest_InitRetouchTaskWithErrorConfig)
{
	//空Config
	{
		CRetouchTask Task;

		EXPECT_ANY_THROW(Task.init(nullptr));
		EXPECT_EQ(Task.getExpander(), nullptr);
		EXPECT_EQ(Task.getClusterConfig(), nullptr);
	}

	//错Config
	{
		const std::string OtherConfigPath = "OtherConfig";
		hiveConfig::CHiveConfig* pConfig = nullptr;
		hiveConfig::hiveParseConfig(OtherConfigPath, hiveConfig::EConfigType::XML, pConfig);
		CRetouchTask Task;

		EXPECT_ANY_THROW(Task.init(pConfig));
		EXPECT_EQ(Task.getExpander(), nullptr);
		EXPECT_EQ(Task.getClusterConfig(), nullptr);
	}
}

TEST(Test_InitPointCloudRetouch, InitPointCloudRetouchManager)
{
	PointCloud_t::Ptr pCloud(new PointCloud_t);
	pcl::PointSurfel t;
	pCloud->push_back(t);
	hiveConfig::CHiveConfig* pConfig = new CPointCloudRetouchConfig;
	hiveConfig::hiveParseConfig("../../UnitTests/Unit_Test_010/PointCloudRetouchConfig.xml", hiveConfig::EConfigType::XML, pConfig);
	auto pManager = CPointCloudRetouchManager::getInstance();

	pManager->init(pCloud, pConfig->findSubconfigByName("Retouch"));
	ASSERT_EQ(pManager->getClusterSet().getNumCluster(), 0);
	ASSERT_EQ(pManager->getLabelSet().getSize(), pCloud->size());
	ASSERT_NE(pManager->getLitterMarker().getExpander(), nullptr);
	ASSERT_NE(pManager->getBackgroundMarker().getExpander(), nullptr);
}
