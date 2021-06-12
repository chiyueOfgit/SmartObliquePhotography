#include "pch.h"
#include "CompositeClassifier.h"
#include "PointCloudAutoRetouchScene.h"

using namespace hiveObliquePhotography::AutoRetouch;

//*****************************************************************
//FUNCTION: 
void ICompositeClassifier::_ensembleResult()
{
	_ASSERTE((m_ClassifierSet.size() > 1) && m_pGlobalLabelSet);

	std::vector<std::vector<SPointLabelChange>::const_iterator> itr;
	for (auto i = 0; i < m_ClassifierSet.size(); i++) itr[i] = m_ClassifierSet[i]->getResult().cbegin();

	std::vector<SPointLabelChange> OverallResult4SinglePoint;
	std::vector<SPointLabelChange> EnsembledResult4GlobalLabel;
	EnsembledResult4GlobalLabel.reserve(CPointCloudAutoRetouchScene::getInstance()->getNumPoint());
	SPointLabelChange t;

	for (auto i = 0; i < CPointCloudAutoRetouchScene::getInstance()->getNumPoint(); i++)
	{
		OverallResult4SinglePoint.clear();
		for (auto k = 0; k < m_ClassifierSet.size(); i++)
		{
			if (i == itr[k]->Index)
			{
				_ASSERTE(itr[k] < m_ClassifierSet[k]->getResult().cend());
				OverallResult4SinglePoint.emplace_back(*itr[k]);
				itr[k]++;
			}
		}

		if (!OverallResult4SinglePoint.empty())
		{
			t = { i, OverallResult4SinglePoint[0].SrcLabel, __ensembleSingleResultV(OverallResult4SinglePoint)};
			EnsembledResult4GlobalLabel.emplace_back(t);
		}
	}
	m_pGlobalLabelSet->applyPointLabelChange(EnsembledResult4GlobalLabel);
}

//*****************************************************************
//FUNCTION: 
void ICompositeClassifier::addClassifier(IPointClassifier* vClassifer)
{
	_ASSERTE(vClassifer);

//TODO：确保vClassfier以前没有被加进来过
	m_ClassifierSet.emplace_back(vClassifer);
}