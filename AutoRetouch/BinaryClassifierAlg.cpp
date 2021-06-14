#include "pch.h"
#include "BinaryClassifierAlg.h"
#include "PointCloudAutoRetouchScene.h"
#include "PointCluster.h"
#include "PointClusterSet.h"

using namespace hiveObliquePhotography::AutoRetouch;

_REGISTER_NORMAL_PRODUCT(CBinaryClassifierAlg, CLASSIFIER_BINARY)

//*****************************************************************
//FUNCTION:
void CBinaryClassifierAlg::runV(const std::string& vClusterType)
{
	if (hiveConfig::hiveParseConfig("AutoRetouchConfig.xml", hiveConfig::EConfigType::XML, CAutoRetouchConfig::getInstance()) != hiveConfig::EParseResult::SUCCEED)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", "AutoRetouchConfig.xml"));
		return;
	}
	
	m_ClusterSet = CPointClusterSet::getInstance()->getGlobalClusterSet(vClusterType);
	_ASSERTE(!m_ClusterSet.empty());
	if (m_ClusterSet.empty())
		return;

	const auto pScene = CPointCloudAutoRetouchScene::getInstance();
	const auto pCloud = pScene->getPointCloudScene();
	const auto pTree = pScene->getGlobalKdTree();
	_ASSERTE(pCloud != nullptr && pTree != nullptr);

	auto UnknownIndices = __getUnknownIndices();

	auto ExecuteArea = __createExecuteArea();

#pragma omp parallel for
	for (int i = 0; i < UnknownIndices->size(); i++)
	{
		const auto Index = UnknownIndices->at(i);
		if (ExecuteArea.isInBox(pCloud->at(Index).x, pCloud->at(Index).y, pCloud->at(Index).z))
		{
			double MaxScore = -FLT_MAX;
			std::size_t MaxIndex = -1;

#pragma omp parallel for
			for (int i = 0; i < m_ClusterSet.size(); i++)
			{
				const auto Score = m_ClusterSet[i]->computeSimilarityV(Index);
				if (Score > MaxScore)
				{
					MaxScore = Score;
					MaxIndex = i;
				}
			}

#pragma omp critical
			if (m_ClusterSet[MaxIndex]->getClusterLabel() == EPointLabel::UNWANTED)
				m_pLocalLabelSet->changePointLabel(Index, m_ClusterSet[MaxIndex]->getClusterLabel());
		}
	}
}

//*****************************************************************
//FUNCTION:
pcl::IndicesPtr CBinaryClassifierAlg::__getUnknownIndices()
{
	std::set<pcl::index_t> CloudIndex;
	for (pcl::index_t i = 0; i < CPointCloudAutoRetouchScene::getInstance()->getPointCloudScene()->size(); ++i)
		CloudIndex.insert(i);

	std::set<pcl::index_t> WholeClusterIndices;
	for (auto pPointCluster : m_ClusterSet)
		WholeClusterIndices.insert(pPointCluster->getClusterIndices()->begin(), pPointCluster->getClusterIndices()->end());

	pcl::IndicesPtr RemainIndex(new pcl::Indices);
	std::set_difference(
		CloudIndex.begin(), CloudIndex.end(), 
		WholeClusterIndices.begin(), WholeClusterIndices.end(), 
		std::inserter(*RemainIndex, RemainIndex->begin()));
	
	return RemainIndex;
}

//*****************************************************************
//FUNCTION: 
SBox CBinaryClassifierAlg::__createExecuteArea() const
{
	auto& AreaBox = CPointClusterSet::getInstance()->getAreaBox();
	auto& SceneBox = CPointCloudAutoRetouchScene::getInstance()->getSceneAABB();

	float Factor = *CAutoRetouchConfig::getInstance()->getAttribute<float>(KEY_WORDS::EXCUTEAREA_EXPAND_RATIO);
	const Eigen::Vector3f Padding = (SceneBox.Max - SceneBox.Min) * Factor;
	return { AreaBox.Min - Padding, AreaBox.Max + Padding };
}