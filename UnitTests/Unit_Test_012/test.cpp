#include "pch.h"
#include "PointCloudRetouchInterface.h"
#include "PointCluster.h"
#include "pcl/io/pcd_io.h"
#include "InitialClusterCreator.h"
#include "PointCloudRetouchConfig.h"


//测试用例列表：
//  * DeathTest_Uninitialized: 未进行初始化就进行Probability计算
//  * DeathTest_InvalidIndex: 计算Probability时索引越界
//  * FalseProbability_Test: 计算的Probability不合规范

using namespace hiveObliquePhotography;

const std::string ConfigPath = "PointCloudRetouchConfig.xml";

class TestPointCluster : public testing::Test
{
public:
    hiveConfig::CHiveConfig* pConfig = new PointCloudRetouch::CPointCloudRetouchConfig;
	const hiveConfig::CHiveConfig* pClusterConfig = nullptr;
protected:
	
	void SetUp() override
	{
		if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, pConfig)!= hiveConfig::EParseResult::SUCCEED)
		{
			_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
			return;
		}
		PointCloud_t::Ptr pCloud(new PointCloud_t);
		pcl::io::loadPCDFile("slice 3.pcd", *pCloud);
		PointCloudRetouch::hiveInit(pCloud, pConfig);

		for (auto i = 0; i < pConfig->getNumSubconfig(); i++)
		{
			const hiveConfig::CHiveConfig* TempConfig = pConfig->getSubconfigAt(i);
			if (_IS_STR_IDENTICAL(TempConfig->getSubconfigType(), std::string("TASK")))
			{
				for (auto k = 0; k < TempConfig->getNumSubconfig(); k++)
				{
					const hiveConfig::CHiveConfig* cConfig = pConfig->getSubconfigAt(k);
					if (_IS_STR_IDENTICAL(cConfig->getSubconfigType(), std::string("CLUSTER")))
					{
						if (!pClusterConfig)
				        {
					        pClusterConfig = cConfig;
				        }
				        else
				        {
					       _HIVE_OUTPUT_WARNING("It is NOT allowed to define cluster twice.");
				        }
					}
					continue;
					
				}
				
				continue;
			}
		}
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
