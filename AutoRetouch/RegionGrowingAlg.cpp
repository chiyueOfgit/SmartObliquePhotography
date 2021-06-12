#include "pch.h"
#include "RegionGrowingAlg.h"
#include "AutoRetouchConfig.h"
#include "PointCloudAutoRetouchScene.h"

using namespace hiveObliquePhotography::AutoRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CRegionGrowingAlg, CLASSIFIER_REGION_GROW)

//*****************************************************************
//FUNCTION: 
void CRegionGrowingAlg::runV(const pcl::Indices& vSeeds, EPointLabel vDstLabel)
{
	constexpr unsigned char DEFAULT_TRAVERSED = 1 << 0;
	constexpr unsigned char NEIGHBOR_TRAVERSED = 1 << 1;
	constexpr unsigned char VALIDATE_TRAVERSED = 1 << 2;

	const auto pCloud = CPointCloudAutoRetouchScene::getInstance()->getPointCloudScene();
	//TODO:使用NeighborhoodBuilder
	const auto pTree = CPointCloudAutoRetouchScene::getInstance()->getGlobalKdTree();
	const auto SearchRadius = *CAutoRetouchConfig::getInstance()->getAttribute<double>("SEARCH_RADIUS");
	//TODO: 确保初始化成功？
	
	std::vector Traversed(pCloud->size(), DEFAULT_TRAVERSED);
	__initValidation(vSeeds, pCloud);

	pcl::Indices Seeds(vSeeds);
	//for (auto CurrentIndex : vSeeds)
	//{
	//	//TRAVERE
	//}

	while (!Seeds.empty())
	{
		const auto CurrentIndex = Seeds.back();
		Seeds.pop_back();

		if (__testAndUpdateMask(Traversed[CurrentIndex], NEIGHBOR_TRAVERSED))
			continue;

		//TODO:使用NeighborhoodBuilder
		pcl::Indices NeighborIndices;
		std::vector<float> NeighborDistances;
		pTree->radiusSearch(pCloud->at(CurrentIndex), SearchRadius, NeighborIndices, NeighborDistances);

		for (auto NeighborIndex : NeighborIndices)
		{
			if (__testAndUpdateMask(Traversed[NeighborIndex], VALIDATE_TRAVERSED))
				continue;

			if (!__validatePointV(NeighborIndex, pCloud))
				continue;

			Seeds.insert(Seeds.end(), NeighborIndices.begin(), NeighborIndices.end());
			for (auto Index : NeighborIndices)
				m_pLocalLabelSet->changePointLabel(Index, vDstLabel);
			break;
		}
	}
}

//*****************************************************************
//FUNCTION: 
template<typename T>
bool CRegionGrowingAlg::__testAndUpdateMask(T& vioSubject, const T& vMask)
{
	if ((vioSubject & vMask) == vMask)
		return true;
	else
		vioSubject |= vMask;
	return false;
}
