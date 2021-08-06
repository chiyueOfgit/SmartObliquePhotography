#include "pch.h"

#include "PointCloudRetouchManager.h"
#include "PointCloudRetouchConfig.h"
#include "PointCloudRetouchInterface.h"
#include "HoleRepairer.h"

//测试用例列表：
//  * Simple_Model: 简单模型的OBB正确性判断


using namespace hiveObliquePhotography::PointCloudRetouch;

const auto CubeModelPath = TESTMODEL_DIR + std::string("Test021_Model/Cube.pcd");
const auto ConfigPath = TESTMODEL_DIR + std::string("Config/Test021_PointCloudRetouchConfig.xml");

class TestOBB : public testing::Test
{
public:
	hiveConfig::CHiveConfig* pConfig = nullptr;
	CPointCloudRetouchManager* pManager = nullptr;
	int CloudSize = 0;

protected:
	void SetUp() override
	{
		pConfig = new CPointCloudRetouchConfig;
		if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, pConfig) != hiveConfig::EParseResult::SUCCEED)
		{
			_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
			return;
		}
	}

	void TearDown() override
	{

	}

	void initTest(const std::string& vModelPath)
	{
		PointCloud_t::Ptr pCloud(new PointCloud_t);
		pcl::io::loadPCDFile(vModelPath, *pCloud);
		CloudSize = pCloud->size();
		ASSERT_GT(pCloud->size(), 0);
		pManager = CPointCloudRetouchManager::getInstance();
		pManager->init(pCloud, pConfig);
	}

	void generateCube(pcl::PointCloud<pcl::PointXYZ>::Ptr& vioCloud)
	{
		for (float XAxis = -5; XAxis <= 5; XAxis += 0.1)
			for (float YAxis = -5; YAxis <= 5; YAxis += 0.1)
				for (float ZAxis = -5; ZAxis <= 5; ZAxis += 0.1)
					vioCloud->push_back({XAxis, YAxis, ZAxis});
	}

};

TEST_F(TestOBB, Cube)
{
	initTest(CubeModelPath);
	std::vector<pcl::index_t> Indices;
	for (int i = 0; i < CloudSize; i++)
		Indices.push_back(i);
	std::tuple<Eigen::Matrix3f, Eigen::Vector3f, Eigen::Vector3f> CubeOBB;
	CHoleRepairer Repairer;
	
	CubeOBB = Repairer.calcOBBByIndices(Indices);
	for (int i = 0; i < 3; i++)
	{
		EXPECT_LT(std::get<1>(CubeOBB)[i] - (-5),0.01);
		EXPECT_LT(std::get<2>(CubeOBB)[i] - 5, 0.01);
	}

	bool Result = false;
	Eigen::Matrix3f Axis = std::get<0>(CubeOBB);
	Eigen::Matrix3f GTaxis;
	GTaxis << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
	for (int i = 0; i < 3; i++)
		for (int k = 0; k < 3; k++)
		{
			Result = Axis.col(i).isApprox(GTaxis.col(k));
			if (Result)
				break;
		}

	EXPECT_EQ(Result, true);
}