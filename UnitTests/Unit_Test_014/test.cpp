#include "pch.h"

#include "PointCloudRetouchConfig.h"
#include "PointCloudRetouchInterface.h"
#include "PointCloudRetouchManager.h"
#include "PointCluster.h"
#include "PointClusterExpanderMultithread.h"
#include "VisualizationInterface.h"
#include "PointCloudVisualizer.h"

#include "pcl/io/pcd_io.h"


//测试用例列表：
//  * DeathTest_EmptyInput:尝试输入空的集合；
//  * DeathTest_NullptrInput:尝试输入空集合指针；
//	* No_RepeatIndex_Test: 生成的初始CandidateQueue中不应该有输入的Cluster的索引

using namespace  hiveObliquePhotography::PointCloudRetouch;

const std::string ConfigPath = TESTMODEL_DIR + std::string("Config/Test014_PointCloudRetouchConfig.xml");
const std::string ModelPath = TESTMODEL_DIR + std::string("General/slice 16.pcd");
const std::string CameraPath = TESTMODEL_DIR + std::string("Test008_Model/CompleteBuildingCameraInfo.txt");

class TestExpander : public testing::Test
{
public:
	hiveConfig::CHiveConfig* pConfig = nullptr;
	CPointCloudRetouchManager* pManager = nullptr;
	pcl::visualization::PCLVisualizer* pVisualizer = nullptr;
	Eigen::Matrix4d PV;

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

		pVisualizer = new pcl::visualization::PCLVisualizer("Viewer", true);
		pcl::visualization::Camera Camera;
		pVisualizer->loadCameraParameters(CameraPath);
		pVisualizer->getCameraParameters(Camera);
		Eigen::Matrix4d Proj, View;
		Camera.computeProjectionMatrix(Proj);
		Camera.computeViewMatrix(View);
		PV = Proj * View;
	}

	void TearDown() override
	{

	}
};

TEST_F(TestExpander, NoRepeatIndex)
{
	CPointClusterExpander* pPointClusterExpander = new CPointClusterExpander;

	std::vector<pcl::index_t> UserMarkedRegion{ 1,2,3,4 };
	auto UserSpecifiedCluster = pManager->generateInitialCluster(UserMarkedRegion, PV, [](auto) { return 1; }, EPointLabel::KEPT);

	std::queue<pcl::index_t> CandidateQueue = pPointClusterExpander->initExpandingCandidateQueue(UserSpecifiedCluster);
	int Sum = 0;
	while (!CandidateQueue.empty())
	{
		pcl::index_t Index = CandidateQueue.front();
		CandidateQueue.pop();
		if (std::find(UserSpecifiedCluster->getCoreRegion().begin(), UserSpecifiedCluster->getCoreRegion().end(), Index) != UserSpecifiedCluster->getCoreRegion().end())
			Sum++;
	}
	ASSERT_EQ(Sum, 0);
}

TEST_F(TestExpander, EmptyInput)
{
	CPointClusterExpander* pPointClusterExpander = new CPointClusterExpander;

	std::vector<pcl::index_t> UserMarkedRegion{};
	auto UserSpecifiedCluster = pManager->generateInitialCluster(UserMarkedRegion, PV, [](auto) { return 1; }, EPointLabel::KEPT);
	
	EXPECT_ANY_THROW(pPointClusterExpander->execute<CPointClusterExpander>(UserSpecifiedCluster));
}

TEST_F(TestExpander, NullptrInput)
{
	CPointClusterExpander* pPointClusterExpander = new CPointClusterExpander;
	CPointCluster* UserSpecifiedCluster = nullptr;

	EXPECT_ANY_THROW(pPointClusterExpander->execute<CPointClusterExpander>(UserSpecifiedCluster));
}