#include "pch.h"
#include "BinaryClassifierAlgByVFH.h"
#include "PointCloudAutoRetouchScene.h"
#include "PointClusterSet.h"
#include "PointCluster4VFH.h"

using namespace hiveObliquePhotography::AutoRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CBinaryClassifierByVFHAlg, CLASSIFIER_BINARY_VFH)

//*****************************************************************
//FUNCTION: 
void CBinaryClassifierByVFHAlg::runV()
{
	m_ClusterSet = CPointClusterSet::getInstance()->getGlobalClusterSet();
	_ASSERTE(!m_ClusterSet.empty());

	auto pScene = CPointCloudAutoRetouchScene::getInstance();
	_ASSERTE(pScene);

	auto pCloud = pScene->getPointCloudScene();
	auto pTree = pScene->getGlobalKdTree();
	_ASSERTE(pCloud && pTree);

	auto RemainIndex = __getRemainIndex();

	m_AABB = CPointClusterSet::getInstance()->getAreaBox();
	auto SceneAABB = pScene->getSceneAABB();

	const Eigen::Vector3f Padding = (SceneAABB.Max - SceneAABB.Min) * 0.15f;
	SBox ExecuteAreaWithDelta({ m_AABB.Min - Padding, m_AABB.Max + Padding });

	for (auto Index : RemainIndex)
	{
		if (ExecuteAreaWithDelta.isInBox((*pCloud)[Index].x, (*pCloud)[Index].y, (*pCloud)[Index].z))
		{
			std::pair<double, std::uint64_t> MaxRecord{ -FLT_MAX, -1 };
				for (int i = 0; i < m_ClusterSet.size(); i++)
				{
					auto Score = m_ClusterSet[i]->computeDistanceV(Index);
						if (Score > MaxRecord.first)
						{
							MaxRecord.first = Score;
								MaxRecord.second = i;
						}
				}

			m_pLocalLabelSet->changePointLabel(Index, dynamic_cast<CPointCluster4VFH*>(m_ClusterSet[MaxRecord.second])->getClusterLabel());
		}
	}
}

std::set<std::uint64_t> CBinaryClassifierByVFHAlg::__getRemainIndex()
{
	std::set<std::uint64_t> CloudIndex;
	for (std::uint64_t i = 0; i < CPointCloudAutoRetouchScene::getInstance()->getPointCloudScene()->size(); i++)
		CloudIndex.insert(i);

	std::set<std::uint64_t> RemainIndex;
	std::vector<std::set<std::size_t>> WholeClusterIndices;
	for (auto pPointCluster : m_ClusterSet)
		WholeClusterIndices.push_back(dynamic_cast<CPointCluster4VFH*>(pPointCluster)->getClusterIndices());

	if (WholeClusterIndices.size() == 1)
	{
		std::set_difference(CloudIndex.begin(), CloudIndex.end(), WholeClusterIndices[0].begin(), WholeClusterIndices[0].end(), std::inserter(RemainIndex, RemainIndex.begin()));
	}
	else
	{
		std::set<std::size_t> UnionIndex = WholeClusterIndices[0];
		for (int i = 1; i < WholeClusterIndices.size(); i++)
		{
			std::set<std::size_t> TempIndex;
			std::set_union(UnionIndex.begin(), UnionIndex.end(), WholeClusterIndices[i].begin(), WholeClusterIndices[i].end(), std::inserter(TempIndex, TempIndex.begin()));
			UnionIndex = TempIndex;
		}

		std::set_difference(CloudIndex.begin(), CloudIndex.end(), UnionIndex.begin(), UnionIndex.end(), std::inserter(RemainIndex, RemainIndex.begin()));
	}

	return RemainIndex;
}