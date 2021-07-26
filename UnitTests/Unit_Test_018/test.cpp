#include "pch.h"
#include "PointCloudRetouchInterface.h"
#include "PointCloudRetouchManager.h"
#include "PointCloudRetouchConfig.h"
#include "ObliquePhotographyDataInterface.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

const auto ConfigFile = TESTMODEL_DIR + std::string("Config/Test018_PointCloudRetouchConfig.xml");
const auto DataPath = TESTMODEL_DIR + std::string("Test018_Model/");

const std::vector<std::string> ModelNames{ "one_hole", "five_holes" };

class TestBoundaryDetection : public testing::Test
{
protected:
	void SetUp() override
	{
		m_pConfig = new CPointCloudRetouchConfig;
		ASSERT_EQ(hiveConfig::hiveParseConfig(ConfigFile, hiveConfig::EConfigType::XML, m_pConfig), hiveConfig::EParseResult::SUCCEED);

		//std::string ModelPath(TESTMODEL_DIR + std::string("General/slice 16.pcd"));
		//PointCloud_t::Ptr pCloud(new PointCloud_t);
		//pcl::io::loadPLYFile(ModelPath, *pCloud);
		//m_pManager = CPointCloudRetouchManager::getInstance();
		//m_pManager->init(pCloud, m_pConfig);
	}

	void TearDown() override
	{
		delete m_pConfig;
	}

	hiveConfig::CHiveConfig* m_pConfig = nullptr;
	CPointCloudRetouchManager* m_pManager = nullptr;
	static int m_TestNumber;
private:
};

int TestBoundaryDetection::m_TestNumber = 0;

TEST(TestCaseName, TestName)
{
  EXPECT_EQ(1, 1);
  EXPECT_TRUE(true);
}