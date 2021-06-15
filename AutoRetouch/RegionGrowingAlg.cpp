#include "pch.h"
#include "RegionGrowingAlg.h"
#include "PointCloudAutoRetouchScene.h"

using namespace hiveObliquePhotography::AutoRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CRegionGrowingAlg, CLASSIFIER_REGION_GROW)

//*****************************************************************
//FUNCTION: 
void CRegionGrowingAlg::runV(const pcl::Indices& vioPoints, EPointLabel vDstLabel)
{
	constexpr unsigned char DEFAULT_TRAVERSED = 1 << 0;
	constexpr unsigned char NEIGHBOR_TRAVERSED = 1 << 1;
	constexpr unsigned char VALIDATE_TRAVERSED = 1 << 2;

	if (vioPoints.empty())
		return;
	
	const auto pCloud = CPointCloudAutoRetouchScene::getInstance()->getPointCloudScene();
	const auto pTree = CPointCloudAutoRetouchScene::getInstance()->getGlobalKdTree();
	for (auto CurrentIndex : vioPoints)
		if (CurrentIndex < 0 || CurrentIndex >= pCloud->size())
			_THROW_RUNTIME_ERROR("Index is out of range");
	
	std::vector Traversed(pCloud->size(), DEFAULT_TRAVERSED);
	__initValidation(vioPoints, pCloud);
	
	for (auto Index : vioPoints)
		m_pLocalLabelSet->changePointLabel(Index, vDstLabel);

	const auto SearchRadius = *CAutoRetouchConfig::getInstance()->getAttribute<double>(KEY_WORDS::SEARCH_RADIUS);
	pcl::Indices Seeds(vioPoints);
	while (!Seeds.empty())
	{
		const auto CurrentIndex = Seeds.back();
		Seeds.pop_back();

		if (__testAndUpdateMask(Traversed[CurrentIndex], NEIGHBOR_TRAVERSED))
			continue;

		//TODO: π”√NeighborhoodBuilder
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
