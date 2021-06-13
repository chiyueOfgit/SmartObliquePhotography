#include "pch.h"
#include "BinaryClassifierAlg.h"
#include "PointCloudAutoRetouchScene.h"
#include "PointCluster.h"
#include "PointClusterSet.h"
#include <omp.h>

using namespace hiveObliquePhotography::AutoRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CBinaryClassifierAlg, CLASSIFIER_BINARY)

//*****************************************************************
//FUNCTION:
void CBinaryClassifierAlg::runV()
{
	m_ClusterSet = CPointClusterSet::getInstance()->getGlobalClusterSet();
	_ASSERTE(!m_ClusterSet.empty());

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
				const auto Score = m_ClusterSet[i]->computeDistanceV(Index);
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
	//TODO：添加至配置文件，可执行区域外沿的宽度与场景大小的比例
	const Eigen::Vector3f Padding = (SceneBox.Max - SceneBox.Min) * 0.15f;
	return { AreaBox.Min - Padding, AreaBox.Max + Padding };
}