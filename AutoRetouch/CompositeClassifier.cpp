#include "pch.h"
#include "PointCloudAutoRetouchScene.h"
#include "CompositeClassifier.h"

using namespace hiveObliquePhotography::AutoRetouch;

//*****************************************************************
//FUNCTION: 
void CCompositeClassifier::__ensembleResult()
{
	_ASSERTE((m_ClassifierSet.size() > 1) && m_pGlobalLabelSet);

	std::vector<std::vector<SPointLabelChange>::const_iterator> itr;
	itr.resize(m_ClassifierSet.size());

	for (auto i = 0; i < m_ClassifierSet.size(); i++)
		itr[i] = m_ClassifierSet[i]->getResult().cbegin();

	std::vector<SPointLabelChange> OverallResult4SinglePoint;
	std::vector<SPointLabelChange> EnsembledResult4GlobalLabel;
	EnsembledResult4GlobalLabel.reserve(CPointCloudAutoRetouchScene::getInstance()->getNumPoint());

	for (auto i = 0; i < CPointCloudAutoRetouchScene::getInstance()->getNumPoint(); i++)
	{
		OverallResult4SinglePoint.clear();
		for (auto k = 0; k < m_ClassifierSet.size(); k++)
		{
			if (itr[k] != m_ClassifierSet[k]->getResult().cend() && i == itr[k]->Index)
			{
				_ASSERTE(itr[k] != m_ClassifierSet[k]->getResult().cend());
				OverallResult4SinglePoint.push_back(*itr[k]);
				++itr[k];
			}
			else if (m_pGlobalLabelSet->getPointLabel(i) == EPointLabel::UNDETERMINED)
			{
				OverallResult4SinglePoint.push_back({ i, m_pGlobalLabelSet->getPointLabel(i), m_pGlobalLabelSet->getPointLabel(i) });
			}
		}

		if (!OverallResult4SinglePoint.empty())
		{
			SPointLabelChange Temp = { i, OverallResult4SinglePoint[0].SrcLabel, __ensembleSingleResultV(OverallResult4SinglePoint) };
			if (Temp.SrcLabel != Temp.DstLabel)
				EnsembledResult4GlobalLabel.push_back(Temp);
		}
	}
	m_pGlobalLabelSet->applyPointLabelChange(EnsembledResult4GlobalLabel, false);
}

//*****************************************************************
//FUNCTION: 
EPointLabel CCompositeClassifier::__ensembleSingleResultV(const std::vector<SPointLabelChange>& vOverallResult) const
{
	for (auto& Result : vOverallResult)
	{
		if (Result.DstLabel == EPointLabel::UNWANTED)
			return EPointLabel::UNWANTED;
	}
	return EPointLabel::UNDETERMINED;
}

//*****************************************************************
//FUNCTION: 
bool CCompositeClassifier::onProductCreatedV(CGlobalPointLabelSet* vGlobalLabelSet, const std::vector<std::string>& vChildClassifierSigSet)
{
	_ASSERTE(vGlobalLabelSet && (vChildClassifierSigSet.size() > 1));
	m_pGlobalLabelSet = vGlobalLabelSet;
	m_pLocalLabelSet = m_pGlobalLabelSet->clone();
	_ASSERTE(m_pLocalLabelSet);

	for (const auto& e : vChildClassifierSigSet)
	{
		IPointClassifier* pClassifer = hiveDesignPattern::hiveGetOrCreateProduct<IPointClassifier>(e, vGlobalLabelSet);
		if (pClassifer)
		{
			m_ClassifierSet.emplace_back(pClassifer);
		}
		else
		{
			_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to create classifier object [%1%] due to invalid signature.", e));
		}
	}
	return true;
}
