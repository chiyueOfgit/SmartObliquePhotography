#include "pch.h"
#include "PointCloudRetouchInterface.h"
#include "PointCluster.h"
#include "pcl/io/pcd_io.h"
#include "InitialClusterCreator.h"
#include "PointCloudRetouchConfig.h"
#include "PointCloudRetouchManager.h"


//测试用例列表：
//  * DeathTest_Uninitialized: 未进行初始化就进行Probability计算
//  * DeathTest_InvalidIndex: 计算Probability时索引越界
//  * FalseProbability_Test: 计算的Probability不合规范

using namespace hiveObliquePhotography;

constexpr char ConfigPath[] = "../../UnitTests/Unit_Test_012/Unit_Test_012Config.xml";

class TestPointCluster : public testing::Test
{
public:
    hiveConfig::CHiveConfig* pConfig = nullptr;
	const hiveConfig::CHiveConfig* pClusterConfig = nullptr;
protected:
	
	void SetUp() override
	{
		PointCloud_t::Ptr pCloud(new PointCloud_t);
		pcl::io::loadPCDFile("slice 3.pcd", *pCloud);
		ASSERT_GT(pCloud->size(), 0);
		
		pConfig = new PointCloudRetouch::CPointCloudRetouchConfig;
		if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, pConfig) != hiveConfig::EParseResult::SUCCEED)
		{
			_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
			return;
		}

		pClusterConfig = pConfig->findSubconfigByName("LitterCluster");
	}

	void TearDown() override
	{
		delete pConfig;
		delete pClusterConfig;
	}
};

TEST_F(TestPointCluster, DeathTest_Uninitialized)
{
	pcl::index_t TestIndex = 1;
	hiveObliquePhotography::PointCloudRetouch::CPointCluster PointCluster;
	ASSERT_ANY_THROW(double Res = PointCluster.evaluateProbability(TestIndex));
}


TEST_F(TestPointCluster, DeathTest_InvalidIndex)
{
	std::vector<pcl::index_t> UserMarkedRegion{1,2,3,4,5,6,7};
	
	pcl::index_t TestIndex = -1;
	Eigen::Matrix4d Pv;
	PointCloudRetouch::CInitialClusterCreator Creator;
	auto PointCluster = Creator.createInitialCluster(UserMarkedRegion, 0.8, 10, PointCloudRetouch::EPointLabel::KEPT, { 400,400 }, Pv, {1000,800}, pClusterConfig);
	ASSERT_ANY_THROW(double Res = PointCluster->evaluateProbability(TestIndex));
}

TEST_F(TestPointCluster, FalseProbability_Test)
{
	std::vector<pcl::index_t> UserMarkedRegion{ 1,2,3,4,5,6,7};
	
	pcl::index_t TestIndex = 1;
	Eigen::Matrix4d Pv;
	PointCloudRetouch::CInitialClusterCreator Creator;
	auto PointCluster = Creator.createInitialCluster(UserMarkedRegion, 0.8, 10, PointCloudRetouch::EPointLabel::KEPT, { 400,400 }, Pv, { 1000,800 }, pClusterConfig);
	double Res = PointCluster->evaluateProbability(TestIndex);
	EXPECT_LE(Res, 1.0);
	EXPECT_GE(Res, 0.0);
}
