#include "pch.h"
#include "AutoRetouchInterface.h"
#include "VisualizationInterface.h"
#include "PointCloudAutoRetouchScene.h"
#include "PointCloudVisualizer.h"
#include "pcl/io/pcd_io.h"

#include <fstream>
#include <boost/archive/text_oarchive.hpp> 
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

using namespace hiveObliquePhotography::AutoRetouch;

//测试用例列表：
//  * Cluster4VFH: 能够正确构造并正确填充信息
//  * BinaryAlgByVFH: 能够用提供好的cluster得到语法正确的结果
//  * CreateClusterByClusterAlg: 能够用聚类算法的结果正确创建cluster

const bool bIsEnableVisualizer = false;

const std::string g_Folder = "test_tile16/";
const std::string g_CloudFile = "Scu_Tile16.pcd";
const std::string g_UnwantedTreePoints = "SomeBigTreePoints.txt";
const std::string g_KeptGroundPoints = "SomeGroundPoints.txt";

class CTestBinary : public testing::Test
{
protected:
	void SetUp() override
	{
		//init scene
		m_pCloud.reset(new pcl::PointCloud<pcl::PointSurfel>);
		pcl::io::loadPCDFile(g_Folder + g_CloudFile, *m_pCloud);

		hiveObliquePhotography::AutoRetouch::CPointCloudAutoRetouchScene::getInstance()->init(m_pCloud);

		//if (bIsEnableVisualizer)
		//{
		//	hiveObliquePhotography::Visualization::CPointCloudVisualizer::getInstance()->init(m_pCloud);
		//	hiveObliquePhotography::Visualization::CPointCloudVisualizer::getInstance()->refresh();
		//}

		createTestcaseContext();
	}
	
	void TearDown() override
	{
		//if (bIsEnableVisualizer)
		//{
		//	hiveObliquePhotography::Visualization::CPointCloudVisualizer::getInstance()->refresh();
		//	system("pause");
		//}
	}

	template<class T>
	std::vector<T> loadPointIndices(const std::string& vPath)
	{
		std::vector<T> Indices;
		std::ifstream File(vPath.c_str());
		boost::archive::text_iarchive ia(File);
		ia >> BOOST_SERIALIZATION_NVP(Indices);
		File.close();	//ia后才能关闭
		return Indices;
	}

	void createTestcaseContext()
	{
		m_UnwantedIndices = loadPointIndices<std::uint64_t>(g_Folder + g_UnwantedTreePoints);
		m_pUnwantedCluster = new CPointCluster4VFH(m_UnwantedIndices, m_Unwanted);

		m_KeptIndices = loadPointIndices<std::uint64_t>(g_Folder + g_KeptGroundPoints);
		m_pKeptCluster = new CPointCluster4VFH(m_KeptIndices, m_Kept);
	}

	std::vector<std::uint64_t> m_UnwantedIndices;
	std::vector<std::uint64_t> m_KeptIndices;
	EPointLabel m_Unwanted = EPointLabel::UNWANTED;
	EPointLabel m_Kept = EPointLabel::KEPT;

	IPointCluster* m_pUnwantedCluster = nullptr;
	IPointCluster* m_pKeptCluster = nullptr;

	pcl::PointCloud<pcl::PointSurfel>::ConstPtr const getCloud() const { return m_pCloud; }

private:
	pcl::PointCloud<pcl::PointSurfel>::Ptr m_pCloud = nullptr;
};

TEST_F(CTestBinary, Cluster4VFH)
{
	ASSERT_EQ(m_pUnwantedCluster->getClusterLabel(), m_Unwanted);

	ASSERT_EQ(m_pKeptCluster->getClusterLabel(), m_Kept);

	std::uint64_t ErrorIndex1 = -1;
	std::uint64_t ErrorIndex2 = getCloud()->size() + 1;

	std::vector<std::uint64_t> ErrorIndices1 = { ErrorIndex1 };
	std::vector<std::uint64_t> ErrorIndices2 = { ErrorIndex2 };

	//无效的创建失败
	EXPECT_DEATH(new CPointCluster4VFH(ErrorIndices1, m_Kept), ".*");
	EXPECT_DEATH(new CPointCluster4VFH(ErrorIndices2, m_Unwanted), ".*");

	//无效的算距离失败
	EXPECT_DEATH(m_pUnwantedCluster->computeDistanceV(ErrorIndex1), ".*");
	EXPECT_DEATH(m_pUnwantedCluster->computeDistanceV(ErrorIndex2), ".*");
	EXPECT_DEATH(m_pKeptCluster->computeDistanceV(ErrorIndex1), ".*");
	EXPECT_DEATH(m_pKeptCluster->computeDistanceV(ErrorIndex2), ".*");

	//簇中的点必须更靠近自己
	for (int i = 0, step = 3; i < m_UnwantedIndices.size() && i < m_KeptIndices.size(); i += step)
	{
		EXPECT_GT(m_pUnwantedCluster->computeDistanceV(m_UnwantedIndices[i]), m_pKeptCluster->computeDistanceV(m_UnwantedIndices[i]));
		EXPECT_GT(m_pKeptCluster->computeDistanceV(m_KeptIndices[i]), m_pUnwantedCluster->computeDistanceV(m_KeptIndices[i]));
	}

}

TEST_F(CTestBinary, BinaryAlgByVFH)
{
	std::vector<IPointCluster*> pClusters = { m_pUnwantedCluster, m_pKeptCluster };

	IPointClassifier* pClassifier = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(CLASSIFIER_BINARY_VFH, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
	ASSERT_NE(pClassifier, nullptr);
	pClassifier->execute<CBinaryClassifierByVFHAlg>(true, pClusters);

	auto LabelChanged = pClassifier->getResult();
	ASSERT_EQ(LabelChanged.size(), getCloud()->size() - m_UnwantedIndices.size() - m_KeptIndices.size());

	//hiveExecuteClusterAlg2CreateCluster(loadPointIndices(g_Folder + g_UnwantedTreePoints), EPointLabel::UNWANTED);
	//hiveExecuteClusterAlg2CreateCluster(loadPointIndices(g_Folder + g_KeptGroundPoints), EPointLabel::KEPT);

	//hiveObliquePhotography::Visualization::hiveInitVisualizer(pCloud);
	//hiveObliquePhotography::Visualization::hiveRefreshVisualizer();

	//hiveObliquePhotography::Visualization::hiveRunVisualizerLoop();
}