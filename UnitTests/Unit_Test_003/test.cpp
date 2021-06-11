#include "pch.h"
#include "AutoRetouchInterface.h"
#include "PointCloudAutoRetouchScene.h"
#include "PointCloudVisualizer.h"
#include <pcl/io/pcd_io.h>

#include <fstream>
#include <boost/archive/text_oarchive.hpp> 
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

using namespace hiveObliquePhotography::AutoRetouch;

//	测试用例列表：
//	* RegionGrowingByColor：测试区域生长功能能否正确运行;
//	* DeathTest_EmptyPoint:尝试输入空的种子点集合；
//	* DeathTest_OverIndexIndices:尝试输入越界的种子点集；
//	* DeathTest_AllPoints:尝试输入整个点云作为种子点；

#define Test1  测试生长_右下的大树
#define Test2  测试生长_中间两棵树
#define Test3  测试生长_上面四颗树
#define Test4  测试生长_左上两棵树
#define Test5  测试生长_幼儿园

const std::string g_Folder = "test_tile16/";
const std::string g_CloudFile = "Scu_Tile16.pcd";
const std::string g_UnwantedTreePoints = "SomeBigTreePoints.txt";

class CTestRegionGrow :public testing::Test
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

	std::vector<std::size_t> loadPointIndices(std::string vPath)
	{
		std::vector<std::size_t> Indices;
		std::ifstream File(vPath.c_str());
		boost::archive::text_iarchive ia(File);
		ia >> BOOST_SERIALIZATION_NVP(Indices);
		File.close();	//ia后才能关闭
		return Indices;
	}

	std::vector<std::uint64_t> testRegionGrowByColor(std::vector<std::uint64_t>& vSeedSet)
	{
		clock_t StartTime, FinishTime;
		StartTime = clock();

		IPointClassifier* pClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(CLASSIFIER_REGION_GROW_COLOR, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
		pClassifier->execute<CRegionGrowingByColorAlg>(true, vSeedSet, EPointLabel::UNWANTED);

		FinishTime = clock();
		std::cout << "\n区域生长花费时间：" << (int)(FinishTime - StartTime) << " ms\n";

		pClassifier->getResult();
		for (auto& Record : pClassifier->getResult())
		{
			m_pResult.push_back(Record.Index);
		}

		return m_pResult;
	}

	pcl::PointCloud<pcl::PointSurfel>::Ptr getCloud(){ return m_pCloud; }

	void executeRegionGrowingPercentageTest(std::string vGroundTruthPath, float vExpectedCorrect, float vExpectedError);

private:
	pcl::PointCloud<pcl::PointSurfel>::Ptr m_pCloud = nullptr;

	std::vector<std::uint64_t> m_pResult;
};

void CTestRegionGrow::executeRegionGrowingPercentageTest(std::string vGroundTruthPath, float vExpectedCorrect, float vExpectedError)
{
	std::vector<std::uint64_t> GroundTruthPointSets;
	std::vector<std::uint64_t> Result;

	auto calculatePercentage = [&](float vExpectedCorrectRate,float vExpectedErrorRate)
	{
		std::vector<std::uint64_t> ResultIndices = Result;
		std::vector<std::uint64_t> GroundTruthIndices = GroundTruthPointSets;

		// correct
		std::vector<int> Intersection(GroundTruthIndices.size(), -1);
		std::set_intersection(ResultIndices.begin(), ResultIndices.end(), GroundTruthIndices.begin(), GroundTruthIndices.end(), Intersection.begin());
		std::size_t NumIntersection = std::distance(Intersection.begin(), std::find(Intersection.begin(), Intersection.end(), -1));

		float CorrectRate = NumIntersection / (float)GroundTruthIndices.size() * 100.0f;
		std::cout << "\n正确率：" << CorrectRate << "\n\n";
		EXPECT_GE(CorrectRate, vExpectedCorrectRate);

		//error
		std::vector<int> Difference(ResultIndices.size(), -1);
		std::set_difference(ResultIndices.begin(), ResultIndices.end(), GroundTruthIndices.begin(), GroundTruthIndices.end(), Difference.begin());
		std::size_t NumDifference = std::distance(Difference.begin(), std::find(Difference.begin(), Difference.end(), -1));

		float ErrorRate = NumDifference / (float)GroundTruthIndices.size() * 100.0f;
		std::cout << "\n错误率: " << ErrorRate << std::endl << std::endl;
		EXPECT_LE(ErrorRate, vExpectedErrorRate);
	};

	GroundTruthPointSets = loadPointIndices(vGroundTruthPath);

	const std::size_t InputCount = 10;

	std::vector<std::uint64_t> InputIndices;
	{
		auto Iter = GroundTruthPointSets.begin();
		for (int i = 0; i < InputCount && Iter != GroundTruthPointSets.end(); i++, Iter++)
		{
			InputIndices.push_back(*Iter);
		}
	}

	Result = testRegionGrowByColor(InputIndices);

	calculatePercentage(vExpectedCorrect, vExpectedError);
}


//TEST_F(CTestRegionGrow, DeathTest_EmptyPoint)
//{
//	std::vector<std::uint64_t> SeedSet = { };
//	EXPECT_DEATH(hiveObliquePhotography::AutoRetouch::hiveExecuteRegionGrowClassifier(hiveObliquePhotography::AutoRetouch::CLASSIFIER_REGION_GROW_COLOR, SeedSet, EPointLabel::UNWANTED), ".*");
//}
//
//TEST_F(CTestRegionGrow, DeathTest_OverIndexIndices)
//{
//	std::vector<std::uint64_t> SeedSet = {170000,180000 };
//	EXPECT_DEATH(hiveObliquePhotography::AutoRetouch::hiveExecuteRegionGrowClassifier(hiveObliquePhotography::AutoRetouch::CLASSIFIER_REGION_GROW_COLOR, SeedSet, EPointLabel::UNWANTED), ".*");
//
//}

TEST_F(CTestRegionGrow, Test1)
{
	executeRegionGrowingPercentageTest("groundtruth/LeftBigTree.txt", 70.0, 10.0);
}

TEST_F(CTestRegionGrow, Test2)
{
	executeRegionGrowingPercentageTest("groundtruth/MidTwoTrees.txt", 70.0, 10.0);
}

TEST_F(CTestRegionGrow, Test3)
{
	executeRegionGrowingPercentageTest("groundtruth/RightFourTrees.txt", 70.0, 10.0);
}

TEST_F(CTestRegionGrow, Test4)
{
	executeRegionGrowingPercentageTest("groundtruth/RightUpTrees.txt", 70.0, 10.0);
}

TEST_F(CTestRegionGrow, Test5)
{
	executeRegionGrowingPercentageTest("groundtruth/Kindergarten.txt", 10.0, 10.0);
}

TEST_F(CTestRegionGrow, RegionGrowingByColor)
{
	std::vector<std::uint64_t> SeedSet = { 18,19,20,21,22,23,24,25 };

	hiveObliquePhotography::AutoRetouch::hiveExecuteRegionGrowClassifier(hiveObliquePhotography::AutoRetouch::CLASSIFIER_REGION_GROW_COLOR, SeedSet, EPointLabel::UNWANTED);
	
	hiveObliquePhotography::Visualization::CPointCloudVisualizer::getInstance()->init(getCloud(),false);
	hiveObliquePhotography::Visualization::CPointCloudVisualizer::getInstance()->refresh();

	system("pause");
 }










