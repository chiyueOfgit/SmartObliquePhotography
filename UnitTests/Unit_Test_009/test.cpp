#include "pch.h"
#include "AutoRetouchInterface.h"
#include "PointCloudAutoRetouchScene.h"
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <boost/archive/text_oarchive.hpp> 
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

//测试用例列表：
//  * Add_NullClassifier_Test: 测试加入未建成功的Classifier
//  * Add_NullLastResult_Test: 测试AddClassifier后所得索引结果为空的情况
//  * AB_Cluster4Growing_Test: 测试AB组合（聚类区域生长）的正确性
//  * AA_BinaryCombinationOrder_Test: 测试AA组合（多条件二分）关于加入顺序无关性

using namespace hiveObliquePhotography::AutoRetouch;

const std::string g_Folder = "test_tile16/";
const std::string g_CloudFile = "Scu_Tile16.pcd";


class CTestCompositeClassifier :public testing::Test
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
		/*m_pCloud.reset(new pcl::PointCloud<pcl::PointSurfel>);
		pcl::io::loadPCDFile(g_Folder + g_CloudFile, *m_pCloud);

		hiveObliquePhotography::AutoRetouch::CPointCloudAutoRetouchScene::getInstance()->init(m_pCloud);*/
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
};


TEST_F(CTestCompositeClassifier, Add_NullClassifier_Test) {

	const std::string ClusterAlgSig = CLASSIFIER_MaxVisibilityCluster;
	IPointClassifier* pClusterClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(ClusterAlgSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	Eigen::Vector3f CameraPos{};
	std::vector<Eigen::Matrix4d> Matrices{};
	pcl::IndicesPtr PointIndices = nullptr;
	
	CCompositeClassifier* pCompositeClassifier = new CCompositeClassifier;
	pCompositeClassifier->init(CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	GTEST_TEST_NO_THROW_(pCompositeClassifier->addClassifierAndExecute<CMaxVisibilityClusterAlg>(dynamic_cast<CMaxVisibilityClusterAlg*>(pClusterClassifier), PointIndices, EPointLabel::UNWANTED, CameraPos, Matrices));
}

TEST_F(CTestCompositeClassifier, Add_NullLastResult_Test) {

	const std::string ClusterAlgSig = CLASSIFIER_MaxVisibilityCluster;
	IPointClassifier* pClusterClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(ClusterAlgSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	const std::string GrowingAlgSig = CLASSIFIER_REGION_GROW_COLOR;
	IPointClassifier* pGrowingClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(GrowingAlgSig, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());

	Eigen::Vector3f CameraPos{0,0,0};
	Eigen::Matrix4d MatrixA(4,4);
	Eigen::Matrix4d MatrixB(4,4);
	std::vector<Eigen::Matrix4d> Matrices{ MatrixA ,MatrixB};
	pcl::IndicesPtr PointIndices = nullptr;
	
	CCompositeClassifier* pCompositeClassifier = new CCompositeClassifier;
	pCompositeClassifier->init(CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	pCompositeClassifier->addClassifierAndExecute<CMaxVisibilityClusterAlg>(dynamic_cast<CMaxVisibilityClusterAlg*>(pClusterClassifier), PointIndices, EPointLabel::UNWANTED, CameraPos, Matrices);
	ASSERT_NO_THROW(pCompositeClassifier->addClassifierAndExecuteByLastIndices<CRegionGrowingByColorAlg>(dynamic_cast<CRegionGrowingByColorAlg*>(pGrowingClassifier), EPointLabel::UNWANTED));
}

TEST_F(CTestCompositeClassifier, AB_Cluster4Growing_Test) {

	
}

TEST_F(CTestCompositeClassifier, AA_BinaryCombinationOrder_Test) {


}