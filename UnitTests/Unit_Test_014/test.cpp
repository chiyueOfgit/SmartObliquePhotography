#include "pch.h"

#include "PointCloudRetouchConfig.h"
#include "PointCloudRetouchInterface.h"
#include "PointCloudRetouchManager.h"
#include "PointCluster.h"
#include "PointClusterExpander.h"
#include "pcl/io/pcd_io.h"

//测试用例列表：
//  * DeathTest_EmptyInput:尝试输入空的集合；
//  * DeathTest_NullptrInput:尝试输入空集合指针；
//	* No_RepeatIndex_Test: 生成的初始CandidateQueue中不应该有输入的Cluster的索引

using namespace  hiveObliquePhotography::PointCloudRetouch;

constexpr char ConfigPath[] = "../../UnitTests/Unit_Test_014/PointCloudRetouchConfig.xml";
constexpr char ModelPath[] = "../../UnitTests/Unit_Test_014/slice 3.pcd";

class TestExpander : public testing::Test
{
public:
	hiveConfig::CHiveConfig* pConfig = nullptr;
	CPointCloudRetouchManager* pManager = nullptr;
protected:

	void SetUp() override
	{
		PointCloud_t::Ptr pCloud(new PointCloud_t);
		pcl::io::loadPCDFile(ModelPath, *pCloud);
		ASSERT_GT(pCloud->size(), 0);

		pConfig = new CPointCloudRetouchConfig;
		if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, pConfig) != hiveConfig::EParseResult::SUCCEED)
		{
			_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
			return;
		}
		
		pManager = CPointCloudRetouchManager::getInstance();
		pManager->init(pCloud, pConfig);
;	}

	void TearDown() override
	{
		delete pConfig;
	}
};

TEST_F(TestExpander, EmptyInput)
{
	CPointClusterExpander* pPointClusterExpander = new CPointClusterExpander;

	std::vector<pcl::index_t> UserMarkedRegion{};
	Eigen::Matrix4d Pv;
	auto UserSpecifiedCluster = pManager->generateInitialCluster(UserMarkedRegion, 0.8, 10, { 400,400 }, Pv, { 1000,800 }, EPointLabel::KEPT);
	
	ASSERT_ANY_THROW(pPointClusterExpander->execute<CPointClusterExpander>(UserSpecifiedCluster));
}

TEST_F(TestExpander, NullptrInput)
{
	CPointClusterExpander* pPointClusterExpander = new CPointClusterExpander;
	CPointCluster* UserSpecifiedCluster = nullptr;

	ASSERT_ANY_THROW(pPointClusterExpander->execute<CPointClusterExpander>(UserSpecifiedCluster));
}

TEST_F(TestExpander, NoRepeatIndex)
{
	CPointClusterExpander* pPointClusterExpander = new CPointClusterExpander;

	std::vector<pcl::index_t> UserMarkedRegion{1,2,3,4};
	Eigen::Matrix4d Pv;
	auto UserSpecifiedCluster = pManager->generateInitialCluster(UserMarkedRegion, 0.8, 10, { 400,400 }, Pv, { 1000,800 }, EPointLabel::KEPT);

	std::queue<pcl::index_t> CandidateQueue;
	pPointClusterExpander->initExpandingCandidateQueue(UserSpecifiedCluster, CandidateQueue);
	int Sum = 0;
	while(!CandidateQueue.empty())
	{
		pcl::index_t Index = CandidateQueue.front();
		CandidateQueue.pop();
		if (find(UserSpecifiedCluster->getCoreRegion().begin(), UserSpecifiedCluster->getCoreRegion().end(), Index) != UserSpecifiedCluster->getCoreRegion().end())
			Sum++;
	}
	ASSERT_EQ(Sum, 0);
}