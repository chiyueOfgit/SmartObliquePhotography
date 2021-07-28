#include "pch.h"

#include "PointCloudRetouchConfig.h"
#include "PointCloudRetouchInterface.h"
#include "PointCloudRetouchManager.h"
#include "PointCluster.h"
#include "PointClusterExpanderMultithread.h"
#include "PointClusterExpander.h"
#include "VisualizationInterface.h"
#include "PointCloudVisualizer.h"

#include "pcl/io/pcd_io.h"


//测试用例列表：
//  * DeathTest_EmptyInput:尝试输入空的集合；
//  * DeathTest_NullptrInput:尝试输入空集合指针；
//	* No_RepeatIndex_Test: 生成的初始CandidateQueue中不应该有输入的Cluster的索引
//  * MultithreadvsSinglethread: 单线程和多线程执行生长的效率对比

using namespace  hiveObliquePhotography::PointCloudRetouch;

const std::string SinglethreadConfigPath = TESTMODEL_DIR + std::string("Config/Test014_PointCloudRetouchConfig_Singlethread.xml");
const std::string MultithreadConfigPath = TESTMODEL_DIR + std::string("Config/Test014_PointCloudRetouchConfig_Multithread.xml");
const std::string ModelPath = TESTMODEL_DIR + std::string("General/slice 16.pcd");
const std::string CameraPath = TESTMODEL_DIR + std::string("Test014_Model/CompleteBuildingCameraInfo.txt");
const std::string PickedIndicesPath = TESTMODEL_DIR + std::string("Test014_Model/PickedIndices_Slice2.txt");
const std::string Slice2CameraPath = TESTMODEL_DIR + std::string("Test014_Model/Camera_Slice2.txt");
const std::string Slice2ModelPath = TESTMODEL_DIR + std::string("General/slice 2.pcd");

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
		if (hiveConfig::hiveParseConfig(MultithreadConfigPath, hiveConfig::EConfigType::XML, pConfig) != hiveConfig::EParseResult::SUCCEED)
		{
			_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", MultithreadConfigPath));
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

void loadIndices(const std::string& vPath, pcl::Indices& voIndices)
{
	std::ifstream File(vPath);
	boost::archive::text_iarchive ia(File);
	ia >> BOOST_SERIALIZATION_NVP(voIndices);
	File.close();
}

TEST_F(TestExpander, NoRepeatIndex)
{
	CPointClusterExpanderMultithread* pPointClusterExpanderMultithread = new CPointClusterExpanderMultithread;

	std::vector<pcl::index_t> UserMarkedRegion{ 1,2,3,4 };
	auto UserSpecifiedCluster = pManager->generateInitialCluster(UserMarkedRegion, PV, [](auto) { return 1; }, EPointLabel::KEPT);

	std::vector<pcl::index_t> CandidateQueue = pPointClusterExpanderMultithread->initExpandingCandidateQueue(UserSpecifiedCluster);
	int Sum = 0;
	for (auto Index : CandidateQueue)
	{
		if (std::find(UserSpecifiedCluster->getCoreRegion().begin(), UserSpecifiedCluster->getCoreRegion().end(), Index) != UserSpecifiedCluster->getCoreRegion().end())
			Sum++;
	}
	ASSERT_EQ(Sum, 0);
}

TEST_F(TestExpander, EmptyInput)
{
	CPointClusterExpanderMultithread* pPointClusterExpanderMultithread = new CPointClusterExpanderMultithread;

	std::vector<pcl::index_t> UserMarkedRegion{};
	auto UserSpecifiedCluster = pManager->generateInitialCluster(UserMarkedRegion, PV, [](auto) { return 1; }, EPointLabel::KEPT);
	
	EXPECT_ANY_THROW(pPointClusterExpanderMultithread->execute<CPointClusterExpanderMultithread>(UserSpecifiedCluster));
}

TEST_F(TestExpander, NullptrInput)
{
	CPointClusterExpanderMultithread* pPointClusterExpanderMultithread = new CPointClusterExpanderMultithread;
	CPointCluster* UserSpecifiedCluster = nullptr;

	EXPECT_ANY_THROW(pPointClusterExpanderMultithread->execute<CPointClusterExpanderMultithread>(UserSpecifiedCluster));
}

TEST(TestExpander_2, MultithreadvsSinglethread)
{
	// load PickedIndices
	pcl::Indices PickedIndices;
	loadIndices(PickedIndicesPath, PickedIndices);

	// load Camera
	pcl::visualization::Camera Camera;
	auto pVisualizer = new pcl::visualization::PCLVisualizer("Viewer", true);
	pVisualizer->loadCameraParameters(Slice2CameraPath);
	pVisualizer->getCameraParameters(Camera);
	Eigen::Matrix4d ViewMatrix, ProjectionMatrix;
	Camera.computeViewMatrix(ViewMatrix);
	Camera.computeProjectionMatrix(ProjectionMatrix);

	// load PointCloud
	PointCloud_t::Ptr pCloud(new PointCloud_t);
	pcl::io::loadPCDFile(Slice2ModelPath, *pCloud);

	// load Config
	hiveConfig::CHiveConfig* pSinglethreadConfig = new CPointCloudRetouchConfig;
	hiveConfig::CHiveConfig* pMultithreadConfig = new CPointCloudRetouchConfig;
	if (hiveConfig::hiveParseConfig(SinglethreadConfigPath, hiveConfig::EConfigType::XML, pSinglethreadConfig) != hiveConfig::EParseResult::SUCCEED)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", SinglethreadConfigPath));
		return;
	}
	if (hiveConfig::hiveParseConfig(MultithreadConfigPath, hiveConfig::EConfigType::XML, pMultithreadConfig) != hiveConfig::EParseResult::SUCCEED)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", MultithreadConfigPath));
		return;
	}

	// init HardnessFunc
	double Hardness = 0.8;
	double RadiusOnWindow = 38;
	Eigen::Vector2d CircleCenterOnWindow = { 519, 427 };
	auto HardnessFunc = [=](const Eigen::Vector2d& vPos) -> double
	{
		Eigen::Vector2d PosOnWindow((vPos.x() + 1) * Camera.window_size[0] / 2, (vPos.y() + 1) * Camera.window_size[1] / 2);
		double X = (PosOnWindow - CircleCenterOnWindow).norm() / RadiusOnWindow;
		if (X <= 1.0)
		{
			X -= Hardness;
			if (X < 0)
				return 1.0;
			X /= (1 - Hardness);
			X *= X;
			return X * (X - 2) + 1;
		}
		else
			return 0;
	};

	// init pManager
	auto pManager = CPointCloudRetouchManager::getInstance();
	pManager->init(pCloud, pSinglethreadConfig);

	// execute Marker
	pManager->executeMarker(PickedIndices, ProjectionMatrix * ViewMatrix, HardnessFunc, EPointLabel::KEPT);
	
	double SinglethreadRunTime = pManager->getBackgroundMarker().getExpander()->getRunTime();
	auto SinglethreadExpander = pManager->getBackgroundMarker().getExpander()->getExpandPoints();

	pManager->destroy();
	pManager = CPointCloudRetouchManager::getInstance();
	pManager->init(pCloud, pMultithreadConfig);

	pManager->executeMarker(PickedIndices, ProjectionMatrix * ViewMatrix, HardnessFunc, EPointLabel::KEPT);

	double MultithreadRunTime = pManager->getBackgroundMarker().getExpander()->getRunTime();
	auto MultithreadExpander = pManager->getBackgroundMarker().getExpander()->getExpandPoints();

	SYSTEM_INFO SystemInfo;
	GetSystemInfo(&SystemInfo);
	int CPUNumber = SystemInfo.dwNumberOfProcessors;

	hiveEventLogger::hiveOutputEvent(_FORMAT_STR3("SingleThreadRunTime: %1% \n  MultiThreadRunTime: %2% \n  CPU Number: %3% \n", SinglethreadRunTime, MultithreadRunTime, CPUNumber));

	std::sort(MultithreadExpander.begin(), MultithreadExpander.end());
	std::sort(SinglethreadExpander.begin(), SinglethreadExpander.end());
	std::vector<std::size_t> SymmetricDifference;
	std::set_symmetric_difference(MultithreadExpander.begin(), MultithreadExpander.end(),
		SinglethreadExpander.begin(), SinglethreadExpander.end(),
		std::inserter(SymmetricDifference, SymmetricDifference.begin()));

	hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("SingleThread Expander Size: %1% \n  MultiThread Expander Size: %2% \n", SinglethreadExpander.size(), MultithreadExpander.size()));
	hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("Number of different Points between SingleThread Expander and MultiThread Expander: %1% \n", SymmetricDifference.size()));

	EXPECT_EQ(SymmetricDifference.size(), 0);
	EXPECT_LT(MultithreadRunTime, SinglethreadRunTime);
}