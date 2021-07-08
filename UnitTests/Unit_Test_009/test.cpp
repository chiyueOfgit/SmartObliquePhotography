#include "pch.h"

#include "PointCloudRetouchConfig.h"
#include "PointCloudRetouchInterface.h"
#include "PointCloudRetouchManager.h"
#include "PointCluster.h"
#include "PointClusterExpander.h"
#include "pcl/io/pcd_io.h"

//²âÊÔÓÃÀýÁÐ±í£º
//  * DeathTest_InvalidInput
//  * FunctionTest_Test1
//	* FunctionTest_Test2
//	* FunctionTest_Test3

using namespace  hiveObliquePhotography::PointCloudRetouch;

constexpr char ConfigPath[] = "PointCloudRetouchConfig.xml";
constexpr char TestOneModelPath[] = "../TestModel/Test009_Model/test1.pcd";
constexpr char TestTwoModelPath[] = "../TestModel/Test009_Model/test2.pcd";
constexpr char TestThreeModelPath[] = "../TestModel/Test009_Model/test3.pcd";

class TestOutlierDetector : public testing::Test
{
public:
	hiveConfig::CHiveConfig* pConfig = nullptr;
	CPointCloudRetouchManager* pManager = nullptr;
protected:

	void SetUp() override
	{
		
		pConfig = new CPointCloudRetouchConfig;
		if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, pConfig) != hiveConfig::EParseResult::SUCCEED)
		{
			_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
			return;
		}
;	}

	void initTest(std::string& ModelPath)
	{
		PointCloud_t::Ptr pCloud(new PointCloud_t);
		pcl::io::loadPCDFile(ModelPath, *pCloud);
		ASSERT_GT(pCloud->size(), 0);
		pManager = CPointCloudRetouchManager::getInstance();
		pManager->init(pCloud, pConfig);
	}

	void TearDown() override
	{
		delete pConfig;
		delete pManager;
	}
};

