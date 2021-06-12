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

//	* DeathTest_EmptyInput:尝试输入空的种子点集合；
//	* DeathTest_IllegalInput:尝试输入越界的种子点集；
//	* RegionGrowingCorrectness:测试区域生长功能生长结果的逻辑正确性(通过反生长剩余点);
//	* RegionGrowingAccuracy:测试区域生长功能生长的完整程度,用于对比算法优劣;
//	* RegionGrowingErrorRate:测试区域生长功能生长的错误生长程度,用于对比算法优劣;
//	* Other_DeathTest_1:

// GroundTruth: tile16 左侧大树、中间两树、右侧四树、后面滑梯

const std::string TestGrowingAlg = CLASSIFIER_REGION_GROW_COLOR;

const std::string g_Folder = "test_tile16/";
const std::string g_CloudFile = "Scu_Tile16.pcd";


class CTestRegionGrow :public testing::Test
{
protected:

    const std::vector<std::string> GroundTruth = {
		"groundtruth/LeftBigTree.txt",
		"groundtruth/MidTwoTrees.txt",
		"groundtruth/RightFourTrees.txt",
		"groundtruth/KinderGarten.txt"
    };

	
	void SetUp() override
	{
		m_pCloud.reset(new pcl::PointCloud<pcl::PointSurfel>);
		pcl::io::loadPCDFile(g_Folder + g_CloudFile, *m_pCloud);

		hiveObliquePhotography::AutoRetouch::CPointCloudAutoRetouchScene::getInstance()->init(m_pCloud);
	}

	void TearDown() override
	{

	}

	pcl::Indices loadPointIndices(std::string vPath)
	{
		pcl::Indices Indices;
		std::ifstream File(vPath.c_str());
		boost::archive::text_iarchive ia(File);
		ia >> BOOST_SERIALIZATION_NVP(Indices);
		File.close();	//ia后才能关闭
		return Indices;
	}

	pcl::Indices getRegionGrowingResult(const pcl::Indices& vSeedSet);
	pcl::PointCloud<pcl::PointSurfel>::Ptr getCloud(){ return m_pCloud; }
	void calcRegionGrowingAccuracy(const std::vector<std::string>& vGroundTruthPath, float vExpectedCorrect);
	void calcRegionGrowingErrorRate(const std::vector<std::string>& vGroundTruthPath, float vExpectedError);

private:
	pcl::PointCloud<pcl::PointSurfel>::Ptr m_pCloud = nullptr;
};

pcl::Indices CTestRegionGrow::getRegionGrowingResult(const pcl::Indices& vSeedSet)
{
	pcl::Indices Result;
	clock_t StartTime, FinishTime;
	StartTime = clock();

	IPointClassifier* pClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(TestGrowingAlg, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	pClassifier->execute<CRegionGrowingAlg>(true, vSeedSet, EPointLabel::UNWANTED);

	FinishTime = clock();
	std::cout << "\n区域生长花费时间：" << (int)(FinishTime - StartTime) << " ms\n";

	pClassifier->getResult();
	for (auto& Record : pClassifier->getResult())
	{
		Result.push_back(Record.Index);
	}
	return Result;
}

void CTestRegionGrow::calcRegionGrowingAccuracy(const std::vector<std::string>& vGroundTruthPath, float vExpectedCorrect)
{
	pcl::Indices GroundTruthPointSets;
	pcl::Indices Result;

	auto calculatePercentage = [&](float vExpectedCorrectRate)
	{
		pcl::Indices ResultIndices = Result;
		pcl::Indices GroundTruthIndices = GroundTruthPointSets;

		std::vector<int> Intersection(GroundTruthIndices.size(), -1);
		std::set_intersection(ResultIndices.begin(), ResultIndices.end(), GroundTruthIndices.begin(), GroundTruthIndices.end(), Intersection.begin());
		std::size_t NumIntersection = std::distance(Intersection.begin(), std::find(Intersection.begin(), Intersection.end(), -1));

		float CorrectRate = NumIntersection / (float)GroundTruthIndices.size() * 100.0f;
		std::cout << "\n正确率：" << CorrectRate << "\n\n";
		EXPECT_GE(CorrectRate, vExpectedCorrectRate);
	};
	for (auto& GroundTruthPath : vGroundTruthPath)
	{
		GroundTruthPointSets = loadPointIndices(GroundTruthPath);
		const std::size_t InputCount = 10;
		pcl::Indices InputIndices;
		{
			auto Iter = GroundTruthPointSets.begin();
			for (int i = 0; i < InputCount && Iter != GroundTruthPointSets.end(); i++, Iter++)
			{
				InputIndices.push_back(*Iter);
			}
		}
		Result = getRegionGrowingResult(InputIndices);
		calculatePercentage(vExpectedCorrect);
	}
}

void CTestRegionGrow::calcRegionGrowingErrorRate(const std::vector<std::string>& vGroundTruthPath, float vExpectedError)
{
	pcl::Indices GroundTruthPointSets;
	pcl::Indices Result;

	auto calculatePercentage = [&](float vExpectedErrorRate)
	{
		pcl::Indices ResultIndices = Result;
		pcl::Indices GroundTruthIndices = GroundTruthPointSets;

		std::vector<int> Difference(ResultIndices.size(), -1);
		std::set_difference(ResultIndices.begin(), ResultIndices.end(), GroundTruthIndices.begin(), GroundTruthIndices.end(), Difference.begin());
		std::size_t NumDifference = std::distance(Difference.begin(), std::find(Difference.begin(), Difference.end(), -1));

		float ErrorRate = NumDifference / (float)GroundTruthIndices.size() * 100.0f;
		std::cout << "\n错误率: " << ErrorRate << std::endl << std::endl;
		EXPECT_LE(ErrorRate, vExpectedErrorRate);
	};
	for (auto& GroundTruthPath : vGroundTruthPath)
	{
		GroundTruthPointSets = loadPointIndices(GroundTruthPath);
		const std::size_t InputCount = 10;
		pcl::Indices InputIndices;
		{
			auto Iter = GroundTruthPointSets.begin();
			for (int i = 0; i < InputCount && Iter != GroundTruthPointSets.end(); i++, Iter++)
			{
				InputIndices.push_back(*Iter);
			}
		}
		Result = getRegionGrowingResult(InputIndices);
		calculatePercentage(vExpectedError);
	}
}

TEST_F(CTestRegionGrow, DeathTest_EmptyInput)
{
	pcl::Indices SeedSet = { };
	EXPECT_DEATH(hiveObliquePhotography::AutoRetouch::hiveExecuteRegionGrowClassifier(TestGrowingAlg, SeedSet, EPointLabel::UNWANTED), ".*");
}

TEST_F(CTestRegionGrow, DeathTest_IllegalInput)
{
	pcl::Indices SeedSet = {170000,180000 };
	EXPECT_DEATH(hiveObliquePhotography::AutoRetouch::hiveExecuteRegionGrowClassifier(TestGrowingAlg, SeedSet, EPointLabel::UNWANTED), ".*");

}

TEST_F(CTestRegionGrow, RegionGrowingCorrectness)
{
	for(auto& GroundTruthPath: GroundTruth)
	{
		auto GroundTruthPointSets = loadPointIndices(GroundTruthPath);
		const int InputCount = 5;
		pcl::Indices InputIndices;
		{
			auto Iter = GroundTruthPointSets.begin();
			for (int i = 0; i < InputCount && Iter != GroundTruthPointSets.end(); i++, Iter++)
			{
				InputIndices.push_back(*Iter);
			}
		}
		auto Result = getRegionGrowingResult(InputIndices);
	
		GTEST_ASSERT_LT(Result.size(), GroundTruthPointSets.size());
	    pcl::Indices Residual(GroundTruthPointSets.size(), -1);
	    auto Tail = std::set_difference(Result.begin(), Result.end(), GroundTruthPointSets.begin(), GroundTruthPointSets.end(), Residual.begin());
	    Residual.resize(Tail - Residual.begin());
		
	    auto AntiElection = getRegionGrowingResult(Residual);
		int Sum = 0;
		for(auto Index:InputIndices)
		{
			if (find(AntiElection.begin(), AntiElection.end(), Index) != AntiElection.end())
				Sum++;
		}
		GTEST_ASSERT_EQ(Sum, InputIndices.size());
	}

}

TEST_F(CTestRegionGrow, RegionGrowingAccuracy)
{
	calcRegionGrowingAccuracy(GroundTruth, 70);
}

TEST_F(CTestRegionGrow, RegionGrowingErrorRate)
{
	calcRegionGrowingErrorRate(GroundTruth, 10);
}









