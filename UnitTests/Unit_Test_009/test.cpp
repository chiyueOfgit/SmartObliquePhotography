#include "pch.h"
#include "AutoRetouchInterface.h"
#include "PointCloudAutoRetouchScene.h"
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <boost/archive/text_oarchive.hpp> 
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

#include "CompositeBinaryClassifierAlg.h"
#include "MaxVisibilityClusterAlg.h"
#include "PointCluster4NormalRatio.h"
#include "PointCluster4Score.h"
#include "PointCluster4VFH.h"

//测试用例列表：
//  * Init_NullptrOrEmpty_Test: 测试init异常反应
//  * Add_GlobalLabel_Test: 测试多次addClassifier后不会影响GlobalLabel
//  * AfterRun_UndoQueue_Test: 测试一次组合操作之对应一次Undo操作
//  * Add_NullLastResult_Test: 测试addClassifier后所得索引结果为空的情况
//  * AB_Cluster4Growing_Test: 测试AB组合（聚类区域生长）的正确性
//  * AA_BinaryCombinationOrder_Test: 测试AA组合（多条件二分）关于加入顺序无关性

using namespace hiveObliquePhotography::AutoRetouch;

const std::string g_Folder = "test_tile16/";
const std::string g_CloudFile = "Scu_Tile16.pcd";
const std::string g_WantedFile = "SomeBigTreePoints.txt";
const std::string g_UnWantedFile = "SomeGroundPoints.txt";

const std::vector<std::string> Other_COMPOSITE_BINARY_CONFIG = { BINARY_CLUSTER_NORMAL, BINARY_CLUSTER_VFH, BINARY_CLUSTER_SCORE  };

class CTestCompositeClassifier :public testing::Test
{
protected:

	void SetUp() override
	{
		m_pCloud.reset(new pcl::PointCloud<pcl::PointSurfel>);
		pcl::io::loadPCDFile(g_Folder + g_CloudFile, *m_pCloud);

		hiveObliquePhotography::AutoRetouch::CPointCloudAutoRetouchScene::getInstance()->init(m_pCloud);
		__initCluster();
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
	pcl::PointCloud<pcl::PointSurfel>::Ptr getCloud() { return m_pCloud; }
	void calcRegionGrowingAccuracy(const std::vector<std::string>& vGroundTruthPath, float vExpectedCorrect);
	void calcRegionGrowingErrorRate(const std::vector<std::string>& vGroundTruthPath, float vExpectedError);

private:
	pcl::PointCloud<pcl::PointSurfel>::Ptr m_pCloud = nullptr;
	void __initCluster();
};

void CTestCompositeClassifier::__initCluster()
{
	auto WantedIndex = loadPointIndices(g_Folder + g_WantedFile);
	auto UnWantedIndex = loadPointIndices(g_Folder + g_UnWantedFile);

	{
		std::size_t ClusterId = 0;
		std::vector<std::string> UwNames, WNames;
		std::vector<IPointCluster*> UwpClusters, WpClusters;
		for (auto& Type : COMPOSITE_BINARY_CONFIG)
		{
			if (Type.find(BINARY_CLUSTER_VFH) != std::string::npos)
			{
				UwNames.push_back(BINARY_CLUSTER_VFH + std::to_string(ClusterId));
				UwpClusters.push_back(new CPointCluster4VFH(std::make_shared<pcl::Indices>(UnWantedIndex), EPointLabel::UNWANTED));
			}
			else if (Type.find(BINARY_CLUSTER_SCORE) != std::string::npos)
			{
				UwNames.push_back(BINARY_CLUSTER_SCORE + std::to_string(ClusterId));
				UwpClusters.push_back(new CPointCluster4Score(std::make_shared<pcl::Indices>(UnWantedIndex), EPointLabel::UNWANTED));
			}
			else if (Type.find(BINARY_CLUSTER_NORMAL) != std::string::npos)
			{
				UwNames.push_back(BINARY_CLUSTER_NORMAL + std::to_string(ClusterId));
				UwpClusters.push_back(new CPointCluster4NormalRatio(std::make_shared<pcl::Indices>(UnWantedIndex), EPointLabel::UNWANTED));
			}
		}
		CPointClusterSet::getInstance()->addPointClusters(UwNames, UwpClusters);
		ClusterId++;

		for (auto& Type : COMPOSITE_BINARY_CONFIG)
		{
			if (Type.find(BINARY_CLUSTER_VFH) != std::string::npos)
			{
				WNames.push_back(BINARY_CLUSTER_VFH + std::to_string(ClusterId));
				WpClusters.push_back(new CPointCluster4VFH(std::make_shared<pcl::Indices>(WantedIndex), EPointLabel::KEPT));
			}
			else if (Type.find(BINARY_CLUSTER_SCORE) != std::string::npos)
			{
				WNames.push_back(BINARY_CLUSTER_SCORE + std::to_string(ClusterId));
				WpClusters.push_back(new CPointCluster4Score(std::make_shared<pcl::Indices>(WantedIndex), EPointLabel::KEPT));
			}
			else if (Type.find(BINARY_CLUSTER_NORMAL) != std::string::npos)
			{
				WNames.push_back(BINARY_CLUSTER_NORMAL + std::to_string(ClusterId));
				WpClusters.push_back(new CPointCluster4NormalRatio(std::make_shared<pcl::Indices>(WantedIndex), EPointLabel::KEPT));
			}
		}
		CPointClusterSet::getInstance()->addPointClusters(WNames, WpClusters);
	}
}

//TEST_F(CTestCompositeClassifier, Init_Nullptr_Test) {
//
//	CCompositeClassifier* pCompositeClassifier = new CCompositeClassifier;
//	CGlobalPointLabelSet* NullptrInput = nullptr;
//	CGlobalPointLabelSet* EmptyInput = new CGlobalPointLabelSet;
//	ASSERT_ANY_THROW(pCompositeClassifier->init(NullptrInput));
//	ASSERT_ANY_THROW(pCompositeClassifier->init(EmptyInput));
//}

TEST_F(CTestCompositeClassifier, Add_GlobalLabel_Test) {

	auto pCompositeClassifier = new CCompositeBinaryClassifierAlg;
	pCompositeClassifier->init(CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	std::vector<EPointLabel> FirstGlobalLabel, SecondGlobalLabel;
	hiveGetGlobalPointLabelSet(FirstGlobalLabel);

	pCompositeClassifier->addBinaryClassifiers(COMPOSITE_BINARY_CONFIG);
	hiveGetGlobalPointLabelSet(SecondGlobalLabel);

	std::vector<EPointLabel> Difference(CPointCloudAutoRetouchScene::getInstance()->getNumPoint());
	auto iter = std::set_difference(FirstGlobalLabel.begin(), FirstGlobalLabel.end(), SecondGlobalLabel.begin(), SecondGlobalLabel.end(), Difference.begin());
	Difference.resize(iter - Difference.begin());

	GTEST_ASSERT_EQ(Difference.size(), 0u);
}

TEST_F(CTestCompositeClassifier, AfterRun_UndoQueue_Test) {

	auto BeforeRun = CPointCloudAutoRetouchScene::getInstance()->getNumOfResultQueue();
	auto pCompositeClassifier = new CCompositeBinaryClassifierAlg;
	pCompositeClassifier->init(CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	pCompositeClassifier->addBinaryClassifiers(COMPOSITE_BINARY_CONFIG);
	pCompositeClassifier->run();
	auto AfterRun = CPointCloudAutoRetouchScene::getInstance()->getNumOfResultQueue();

	GTEST_ASSERT_EQ(BeforeRun + 1, AfterRun);
}

TEST_F(CTestCompositeClassifier, Add_NullLastResult_Test) {

	const std::string ClusterAlgSig = CLASSIFIER_MaxVisibilityCluster;
	IPointClassifier* pClusterClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(ClusterAlgSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	const std::string GrowingAlgSig = CLASSIFIER_REGION_GROW_COLOR;
	IPointClassifier* pGrowingClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(GrowingAlgSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());

	Eigen::Vector3f CameraPos{0,0,0};
	Eigen::Matrix4d MatrixV(4,4);
	Eigen::Matrix4d MatrixP(4,4);
	pcl::IndicesPtr PointIndices = nullptr;
	
	CCompositeClassifier* pCompositeClassifier = new CCompositeClassifier;
	pCompositeClassifier->init(CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	pCompositeClassifier->addClassifierAndExecute<CMaxVisibilityClusterAlg>(dynamic_cast<CMaxVisibilityClusterAlg*>(pClusterClassifier), PointIndices, EPointLabel::UNWANTED, CameraPos, MatrixP * MatrixV);
	ASSERT_NO_THROW(pCompositeClassifier->addClassifierAndExecuteByLastIndices<CRegionGrowingByColorAlg>(dynamic_cast<CRegionGrowingByColorAlg*>(pGrowingClassifier), EPointLabel::UNWANTED));
}


TEST_F(CTestCompositeClassifier, AA_BinaryCombinationOrder_Test) {

	std::vector<EPointLabel> FirstGlobalLabel, SecondGlobalLabel;
	
	auto pCompositeClassifier = new CCompositeBinaryClassifierAlg;
	pCompositeClassifier->init(CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	pCompositeClassifier->addBinaryClassifiers(COMPOSITE_BINARY_CONFIG);
	pCompositeClassifier->run();
	
	hiveGetGlobalPointLabelSet(FirstGlobalLabel);
	CPointCloudAutoRetouchScene::getInstance()->resetLabelSet();
	
	auto pOtherCompositeClassifier = new CCompositeBinaryClassifierAlg;
	pOtherCompositeClassifier->init(CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	pOtherCompositeClassifier->addBinaryClassifiers(Other_COMPOSITE_BINARY_CONFIG);
	pOtherCompositeClassifier->run();
	hiveGetGlobalPointLabelSet(SecondGlobalLabel);

	std::vector<EPointLabel> Difference(CPointCloudAutoRetouchScene::getInstance()->getNumPoint());
	auto iter = std::set_difference(FirstGlobalLabel.begin(), FirstGlobalLabel.end(), SecondGlobalLabel.begin(), SecondGlobalLabel.end(), Difference.begin());
	Difference.resize(iter - Difference.begin());

	GTEST_ASSERT_EQ(Difference.size(), 0u);
}