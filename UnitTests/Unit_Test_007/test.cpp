#include "pch.h"
#include "AutoRetouchInterface.h"
#include "PointCloudAutoRetouchScene.h"
#include "PointCloudVisualizer.h"
#include "StatisticalOutlierAlg.h"
#include <pcl/io/pcd_io.h>

#include <fstream>
#include <boost/archive/text_oarchive.hpp> 
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>


using namespace hiveObliquePhotography::AutoRetouch;

//	测试用例列表：
//	* StaOutlierDetectingAlg：测试离群点检测功能能否正确运行;
//	* calculatePercentage:计算输出结果占GroundTruth的比率；

const std::string g_Folder = "groundtruth/";
const std::string g_CloudFile = "test.pcd";

class CTestOutlierDetection :public testing::Test
{
protected:
	void SetUp() override
	{
		m_pCloud.reset(new pcl::PointCloud<pcl::PointSurfel>);
		pcl::io::loadPCDFile(g_Folder + g_CloudFile, *m_pCloud);

		hiveObliquePhotography::AutoRetouch::CPointCloudAutoRetouchScene::getInstance()->init(m_pCloud);
	}

	void TearDown() override
	{

	}

	std::vector<std::uint64_t> loadPointIndices(std::string vPath)
	{
		std::vector<std::uint64_t> Indices;
		std::ifstream File(vPath.c_str());
		boost::archive::text_iarchive ia(File);
		ia >> BOOST_SERIALIZATION_NVP(Indices);
		File.close();	//ia后才能关闭
		return Indices;
	}

	std::vector<std::uint64_t> testOutlierDetection(std::vector<std::uint64_t>& vioPointSet)
	{
		clock_t StartTime, FinishTime;
		StartTime = clock();

		IPointClassifier* pClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(CLASSIFIER_OUTLIER_DETECTION, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
		pClassifier->execute<CStaOutlierDetectingAlg>(true, vioPointSet, EPointLabel::UNWANTED);

		FinishTime = clock();
		std::cout << "\n离群点检测花费时间：" << (int)(FinishTime - StartTime) << " ms\n";

		pClassifier->getResult();
		for (auto& Record : pClassifier->getResult())
		{
			m_pOutlierSet.push_back(Record.Index);
		}

		return m_pOutlierSet;
	}

	pcl::PointCloud<pcl::PointSurfel>::Ptr getCloud() { return m_pCloud; }

	void CTestOutlierDetection::executeOutlierDetectionTest(const std::vector<std::string>& vGroundTruthPaths, const std::vector<std::uint64_t>& vOutlier, float vExpectedCorrectRate);

private:
	pcl::PointCloud<pcl::PointSurfel>::Ptr m_pCloud = nullptr;
	std::vector<std::uint64_t> m_pOutlierSet;
};

void CTestOutlierDetection::executeOutlierDetectionTest(const std::vector<std::string>& vGroundTruthPaths, const std::vector<std::uint64_t>& vOutlier, float vExpectedCorrectRate)
{
	std::vector<std::uint64_t> GroundTruthPointSets;
	std::vector<std::uint64_t> Result;

	auto calculatePercentage = [&](const std::vector<std::uint64_t>& vGroundTruth, float vExpectedCorrectRate)
	{
		std::vector<std::uint64_t> ResultIndices = Result;
		std::vector<std::uint64_t> GroundTruthIndices = vGroundTruth;

		// correct
		std::vector<int> Intersection(GroundTruthIndices.size(), -1);
		std::set_intersection(ResultIndices.begin(), ResultIndices.end(), GroundTruthIndices.begin(), GroundTruthIndices.end(), Intersection.begin());
		std::size_t NumIntersection = std::distance(Intersection.begin(), std::find(Intersection.begin(), Intersection.end(), -1));

		float CorrectRate = NumIntersection / (float)GroundTruthIndices.size() * 100.0f;
		std::cout << "\n正确率：" << CorrectRate << "\n\n";
		EXPECT_GE(CorrectRate, vExpectedCorrectRate);
	};

	for (auto& Path : vGroundTruthPaths)
	{
		std::vector<std::uint64_t> Temp = loadPointIndices(Path);
		GroundTruthPointSets.insert(GroundTruthPointSets.end(), Temp.begin(), Temp.end());
	}

	Result=testOutlierDetection(GroundTruthPointSets);
	calculatePercentage(vOutlier, vExpectedCorrectRate);

}

TEST_F(CTestOutlierDetection, StaOutlierDetectingAlg)
{
	std::vector<std::string> GroundTruthPaths;
	GroundTruthPaths.push_back("groundtruth/indices.txt");

	std::vector<std::size_t> GroundTruthPointSets;
	for (auto& Path : GroundTruthPaths)
	{
		std::vector<std::size_t> Temp = loadPointIndices(Path);
		GroundTruthPointSets.insert(GroundTruthPointSets.end(), Temp.begin(), Temp.end());
	}
	testOutlierDetection(GroundTruthPointSets);

	hiveObliquePhotography::Visualization::CPointCloudVisualizer::getInstance()->init(getCloud(),false);
	hiveObliquePhotography::Visualization::CPointCloudVisualizer::getInstance()->refresh();
}

TEST_F(CTestOutlierDetection, calculatePercentage)
{
	std::vector<std::string> GroundTruthPaths;
	GroundTruthPaths.push_back("groundtruth/indices.txt");

	std::vector<std::uint64_t> Outlier;
	std::vector<std::uint64_t> Temp = loadPointIndices("groundtruth/Outlier.txt");
	Outlier.insert(Outlier.end(), Temp.begin(), Temp.end());

	executeOutlierDetectionTest(GroundTruthPaths, Outlier,30.0);
}



