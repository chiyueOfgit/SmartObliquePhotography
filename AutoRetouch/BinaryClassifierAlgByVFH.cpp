#include "pch.h"
#include "BinaryClassifierAlgByVFH.h"
#include "PointCloudAutoRetouchScene.h"
#include "PointCluster4VFH.h"

using namespace hiveObliquePhotography::AutoRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CBinaryClassifierByVFHAlg, CLASSIFIER_BINARY_VFH)

//*****************************************************************
//FUNCTION: 
void CBinaryClassifierByVFHAlg::runV(const std::vector<IPointCluster*>& vInputClusterSet)
{
	_ASSERTE(vInputClusterSet.empty() || (vInputClusterSet.size() > 1));

	auto pScene = CPointCloudAutoRetouchScene::getInstance();
	auto pCloud = pScene->getPointCloudScene();
	auto pKdTree = pScene->getGlobalKdTree();

	_ASSERTE(pScene && pCloud && pKdTree);

	std::set<std::uint64_t> CloudIndex;
	for (std::uint64_t i = 0; i < pCloud->size(); i++)
		CloudIndex.insert(i);

	std::set<std::uint64_t> RemainIndex;
	std::vector<std::set<std::size_t>> WholeClusterIndices;
	for (auto pPointCluster : vInputClusterSet)
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

	for (auto Index : RemainIndex)
	{
		std::pair<double, std::uint64_t> MaxRecord{-FLT_MAX, -1};
		for (int i = 0; i < vInputClusterSet.size(); i++)
		{
			auto Score = vInputClusterSet[i]->computeDistanceV(Index);
			if (Score > MaxRecord.first)
			{
				MaxRecord.first = Score;
				MaxRecord.second = i;
			}
		}

		m_pLocalLabelSet->changePointLabel(Index, dynamic_cast<CPointCluster4VFH*>(vInputClusterSet[MaxRecord.second])->getClusterLabel());
	}

}