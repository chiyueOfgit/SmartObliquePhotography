#include "pch.h"

#include "OutlierDetector.h"
#include "PointCloudRetouchConfig.h"
#include "PointCloudRetouchInterface.h"
#include "PointCloudRetouchManager.h"
#include "pcl/io/pcd_io.h"
#include <fstream>
#include <boost/archive/text_oarchive.hpp> 
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

//测试用例列表：
//  * DeathTest_InvalidInput：稳定性测试，输入越界索引；
//  * FunctionTest_Test1：功能测试，离群点去除场景一；
//	* FunctionTest_Test2：功能测试，离群点去除场景二；
//	* FunctionTest_Test3：功能测试，离群点去除场景三；

using namespace  hiveObliquePhotography::PointCloudRetouch;

constexpr char ConfigPath[] = "PointCloudRetouchConfig.xml";

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

	void initTest(const std::string& vModelPath)
	{
		PointCloud_t::Ptr pCloud(new PointCloud_t);
		pcl::io::loadPCDFile(vModelPath, *pCloud);
		ASSERT_GT(pCloud->size(), 0);
		pManager = CPointCloudRetouchManager::getInstance();
		pManager->init(pCloud, pConfig);
	}

	void loadIndices(const std::string& vPath, pcl::Indices& voIndices)
	{
		std::ifstream File(vPath);
		boost::archive::text_iarchive ia(File);
		ia >> BOOST_SERIALIZATION_NVP(voIndices);
		File.close();
	}

	void TearDown() override
	{
		delete pConfig;
	}
};


TEST_F(TestOutlierDetector, DeathTest_InvalidInput)
{
	initTest("../TestModel/Test009_Model/test1.pcd");
	pcl::Indices InputIndices;
	InputIndices.push_back(INT_MAX);
	auto pOutlierDetector = dynamic_cast<COutlierDetector*>(hiveDesignPattern::hiveCreateProduct<IPointClassifier>("OUTLIER_DETECTOR"));
	ASSERT_ANY_THROW(pOutlierDetector->execute<COutlierDetector>(InputIndices, EPointLabel::UNWANTED));
	
}


TEST_F(TestOutlierDetector, FunctionTest_Test1)
{
	initTest("../TestModel/Test009_Model/test1.pcd");
	pcl::Indices InputIndices;
	for (int i = 0; i < pManager->getRetouchScene().getNumPoint(); i++)
		InputIndices.push_back(i);
	auto pOutlierDetector = hiveDesignPattern::hiveCreateProduct<COutlierDetector>(KEYWORD::OUTLIER_DETECTOR);
    pOutlierDetector->execute<COutlierDetector>(InputIndices, EPointLabel::UNWANTED);
	
	pcl::Indices OutlierIndices;
	pManager->getIndicesByLabel(OutlierIndices, EPointLabel::UNWANTED);

	pcl::Indices GroundTruth;
	loadIndices("../TestModel/Test009_Model/test1_indices.txt", GroundTruth);

	pcl::Indices Interaction;
	std::set_intersection(OutlierIndices.begin(), OutlierIndices.end(),
		GroundTruth.begin(), GroundTruth.end(),
		std::inserter(Interaction, Interaction.begin()));
	
	GTEST_ASSERT_EQ(Interaction.size(), GroundTruth.size());
}

TEST_F(TestOutlierDetector, FunctionTest_Test2)
{
	initTest("../TestModel/Test009_Model/test2.pcd");
	pcl::Indices InputIndices;
	for (int i = 0; i < pManager->getRetouchScene().getNumPoint(); i++)
		InputIndices.push_back(i);
	auto pOutlierDetector = dynamic_cast<COutlierDetector*>(hiveDesignPattern::hiveCreateProduct<IPointClassifier>(KEYWORD::OUTLIER_DETECTOR));
	pOutlierDetector->execute<COutlierDetector>(InputIndices, EPointLabel::UNWANTED);

	pcl::Indices OutlierIndices;
	pManager->getIndicesByLabel(OutlierIndices, EPointLabel::UNWANTED);

	pcl::Indices GroundTruth;
	loadIndices("../TestModel/Test009_Model/test2_indices.txt", GroundTruth);

	pcl::Indices Interaction;
	std::set_intersection(OutlierIndices.begin(), OutlierIndices.end(),
		GroundTruth.begin(), GroundTruth.end(),
		std::inserter(Interaction, Interaction.begin()));

	GTEST_ASSERT_EQ(Interaction.size(), GroundTruth.size());
}

TEST_F(TestOutlierDetector, FunctionTest_Test3)
{
	initTest("../TestModel/Test009_Model/test3.pcd");
	pcl::Indices InputIndices;
	for (int i = 0; i < pManager->getRetouchScene().getNumPoint(); i++)
		InputIndices.push_back(i);
	auto pOutlierDetector = dynamic_cast<COutlierDetector*>(hiveDesignPattern::hiveCreateProduct<IPointClassifier>(KEYWORD::OUTLIER_DETECTOR));
	pOutlierDetector->execute<COutlierDetector>(InputIndices, EPointLabel::UNWANTED);

	pcl::Indices OutlierIndices;
	pManager->getIndicesByLabel(OutlierIndices, EPointLabel::UNWANTED);

	pcl::Indices GroundTruth;
	loadIndices("../TestModel/Test009_Model/test4_indices.txt", GroundTruth);

	pcl::Indices Interaction;
	std::set_intersection(OutlierIndices.begin(), OutlierIndices.end(),
		GroundTruth.begin(), GroundTruth.end(),
		std::inserter(Interaction, Interaction.begin()));

	GTEST_ASSERT_EQ(Interaction.size(), GroundTruth.size());
}
