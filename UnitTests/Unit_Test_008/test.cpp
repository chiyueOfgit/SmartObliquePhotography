#include "pch.h"
#include "AutoRetouchInterface.h"
#include "VisualizationInterface.h"
#include "PointCluster4VFH.h"
#include "PointCluster4Score.h"
#include "PointCluster4NormalRatio.h"
#include "CompositeClassifier.h"
#include "PointCloudVisualizer.h"
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"

#include <fstream>
#include <boost/archive/text_oarchive.hpp> 
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

using namespace hiveObliquePhotography::AutoRetouch;

//测试用例列表：
//  * ChangeLabel_Undo_Overview_Test: 是否成功撤销对PointLabel的更改
//  * ClusterSet_Undo_Overview_Test: 是否成功撤销对ClusterSet的更改
//
//  * ChangeLabel_Undo_Cleanup_Test: 对PointLabel的撤销不应该影响下次执行的结果
//  * ClusterSet_Undo_Cleanup_Test: 对ClusterSet的撤销不应该影响下次执行的结果
// 
//  * Empty_ResultQueue_Expect_Test: 对空的结果队列进行撤销不应引起异常

const bool bIsEnableVisualizer = false;

const std::string g_Folder = "test_tile16/";
const std::string g_CloudFile = "Scu_Tile16.pcd";
const std::string g_UnwantedTreePoints = "SomeBigTreePoints.txt";
const std::string g_KeptGroundPoints = "SomeGroundPoints.txt";

class CBinaryTest : public testing::Test
{
protected:
	void SetUp() override
	{
		//init scene
		m_pCloud.reset(new pcl::PointCloud<pcl::PointSurfel>);
		pcl::io::loadPCDFile(g_Folder + g_CloudFile, *m_pCloud);

		hiveObliquePhotography::AutoRetouch::CPointCloudAutoRetouchScene::getInstance()->init(m_pCloud);

		if (bIsEnableVisualizer)
		{
			hiveObliquePhotography::Visualization::CPointCloudVisualizer::getInstance()->init(m_pCloud, false);
			hiveObliquePhotography::Visualization::CPointCloudVisualizer::getInstance()->refresh();
		}

		createTestcaseContext();
	}

	void TearDown() override
	{
		if (bIsEnableVisualizer)
		{
			hiveObliquePhotography::Visualization::CPointCloudVisualizer::getInstance()->refresh();
			system("pause");
		}
	}

	pcl::IndicesPtr loadPointIndices(const std::string& vPath)
	{
		pcl::IndicesPtr pIndices(new pcl::Indices);
		std::ifstream File(vPath.c_str());
		boost::archive::text_iarchive ia(File);
		ia >> BOOST_SERIALIZATION_NVP(*pIndices);
		File.close();	//ia后才能关闭
		return pIndices;
	}

	void createTestcaseContext()
	{
		m_pUnwantedIndices = loadPointIndices(g_Folder + g_UnwantedTreePoints);
		m_pUnwantedCluster4VFH = new CPointCluster4VFH(m_pUnwantedIndices, m_Unwanted);
		m_pUnwantedCluster4Score = new CPointCluster4Score(m_pUnwantedIndices, m_Unwanted);
		m_pUnwantedCluster4NormalRatio = new CPointCluster4NormalRatio(m_pUnwantedIndices, m_Unwanted);

		m_pKeptIndices = loadPointIndices(g_Folder + g_KeptGroundPoints);
		m_pKeptCluster4VFH = new CPointCluster4VFH(m_pKeptIndices, m_Kept);
		m_pKeptCluster4Score = new CPointCluster4Score(m_pKeptIndices, m_Kept);
		m_pKeptCluster4NormalRatio = new CPointCluster4NormalRatio(m_pKeptIndices, m_Kept);
	}

	auto* prepareExcute(const std::vector<IPointCluster*>& vClusters) const
	{
		for (std::size_t i = 0; i < vClusters.size(); i++)
			CPointClusterSet::getInstance()->addPointCluster("vfh_" + std::to_string(i), vClusters[i]);

		auto* pClassifier = hiveDesignPattern::hiveCreateProduct<IPointClassifier>(CLASSIFIER_BINARY, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());

		return pClassifier;
	}

	void testResultSize(const IPointClassifier* vClassifier) const
	{
		//结果数目
		auto LabelChanged = vClassifier->getResult();
		ASSERT_GT(LabelChanged.size(), 0);
		ASSERT_LT(LabelChanged.size(), getCloud()->size());
		ASSERT_LE(LabelChanged.size(), getCloud()->size() - m_pUnwantedIndices->size() - m_pKeptIndices->size());
	}
	
	void testChangeLabelBeforeUndo(const IPointClassifier* vClassifier) const
	{
		for (auto& OneChange : vClassifier->getResult())
			ASSERT_EQ(OneChange.DstLabel, vClassifier->getGlobalLabelSet()->getPointLabel(OneChange.Index));
	}

	void testChangeLabelAfterUndo(const IPointClassifier* vClassifier) const
	{
		for (auto& OneChange : vClassifier->getResult())
			ASSERT_EQ(OneChange.SrcLabel, vClassifier->getGlobalLabelSet()->getPointLabel(OneChange.Index));
	}

	void testClusterSetBeforeUndo(const std::vector<IPointCluster*>& vClusters) const
	{
		for (int i = 0; i < 2; i++)
		{
			auto [Name, pCluster] = *CPointClusterSet::getInstance()->getPointClusterMap().find("vfh_" + std::to_string(i));
			auto& Left = vClusters[i]->getClusterIndices();
			auto& Right = pCluster->getClusterIndices();

			//TODO: fixme
			//EXPECT_EQ(Left.get(), Right.get(), "redundant construction for share_ptr");
			ASSERT_EQ(Left->size(), Right->size());
			for (size_t k = 0; k < Left->size(); k++)
				ASSERT_EQ(Left->at(k), Right->at(k));
		}
	}

	void testClusterSetAfterUndo() const
	{
		for (int i = 0; i < 2; i++)
		{
			const auto& Map = CPointClusterSet::getInstance()->getPointClusterMap();
			ASSERT_EQ(Map.find("vfh_" + std::to_string(i)), Map.end());
		}
	}
	
	pcl::IndicesPtr m_pUnwantedIndices;
	pcl::IndicesPtr m_pKeptIndices;
	EPointLabel m_Unwanted = EPointLabel::UNWANTED;
	EPointLabel m_Kept = EPointLabel::KEPT;

	std::vector<pcl::IndicesPtr> m_pRestrictIndices;

	CPointCluster4VFH* m_pUnwantedCluster4VFH = nullptr;
	CPointCluster4VFH* m_pKeptCluster4VFH = nullptr;
	CPointCluster4Score* m_pUnwantedCluster4Score = nullptr;
	CPointCluster4Score* m_pKeptCluster4Score = nullptr;
	CPointCluster4NormalRatio* m_pUnwantedCluster4NormalRatio = nullptr;
	CPointCluster4NormalRatio* m_pKeptCluster4NormalRatio = nullptr;

	pcl::PointCloud<pcl::PointSurfel>::ConstPtr getCloud() const { return m_pCloud; }

private:
	pcl::PointCloud<pcl::PointSurfel>::Ptr m_pCloud = nullptr;
};

TEST_F(CBinaryTest, ChangeLabel_Undo_Overview_Test)
{
	auto pClassifier = prepareExcute({ m_pUnwantedCluster4VFH, m_pKeptCluster4VFH });
	
	pClassifier->execute<CBinaryClassifierAlg>(true, "vfh");
	testResultSize(pClassifier);
	
	testChangeLabelBeforeUndo(pClassifier);
	CPointCloudAutoRetouchScene::getInstance()->undoLastOp();
	testChangeLabelAfterUndo(pClassifier);
}

TEST_F(CBinaryTest, ClusterSet_Undo_Overview_Test)
{
	const std::vector<IPointCluster*> Clusters = { m_pUnwantedCluster4VFH, m_pKeptCluster4VFH };
	auto pClassifier = prepareExcute(Clusters);
	
	pClassifier->execute<CBinaryClassifierAlg>(true, "vfh");
	testResultSize(pClassifier);
	
	testClusterSetBeforeUndo(Clusters);
	CPointCloudAutoRetouchScene::getInstance()->undoLastOp();
	CPointCloudAutoRetouchScene::getInstance()->undoLastOp();
	CPointCloudAutoRetouchScene::getInstance()->undoLastOp();
	testClusterSetAfterUndo();
}

TEST_F(CBinaryTest, ChangeLabel_Undo_Cleanup_Test)
{
	auto pClassifier = prepareExcute({ m_pUnwantedCluster4VFH, m_pKeptCluster4VFH });
	
	pClassifier->execute<CBinaryClassifierAlg>(true, "vfh");
	auto OnceResult = pClassifier->getResultIndices();
	CPointCloudAutoRetouchScene::getInstance()->undoLastOp();
	
	pClassifier->execute<CBinaryClassifierAlg>(true, "vfh");
	auto TwiceResult = pClassifier->getResultIndices();
	testResultSize(pClassifier);

	testChangeLabelBeforeUndo(pClassifier);
	CPointCloudAutoRetouchScene::getInstance()->undoLastOp();
	testChangeLabelAfterUndo(pClassifier);

	ASSERT_EQ(OnceResult->size(), TwiceResult->size());
	for (size_t i = 0; i < OnceResult->size(); i++)
		ASSERT_EQ(OnceResult->at(i), TwiceResult->at(i));
}

TEST_F(CBinaryTest, ClusterSet_Undo_Cleanup_Test)
{
	const std::vector<IPointCluster*> Clusters = { m_pUnwantedCluster4VFH, m_pKeptCluster4VFH };
	auto pClassifier = prepareExcute(Clusters);
	
	pClassifier->execute<CBinaryClassifierAlg>(true, "vfh");
	CPointCloudAutoRetouchScene::getInstance()->undoLastOp();
	auto OnceResult = pClassifier->getResultIndices();
	
	pClassifier->execute<CBinaryClassifierAlg>(true, "vfh");
	auto TwiceResult = pClassifier->getResultIndices();
	testResultSize(pClassifier);

	testClusterSetBeforeUndo(Clusters);
	CPointCloudAutoRetouchScene::getInstance()->undoLastOp();
	CPointCloudAutoRetouchScene::getInstance()->undoLastOp();
	CPointCloudAutoRetouchScene::getInstance()->undoLastOp();
	testClusterSetAfterUndo();
	
	ASSERT_EQ(OnceResult->size(), TwiceResult->size());
	for (size_t i = 0; i < OnceResult->size(); i++)
		ASSERT_EQ(OnceResult->at(i), TwiceResult->at(i));
}
