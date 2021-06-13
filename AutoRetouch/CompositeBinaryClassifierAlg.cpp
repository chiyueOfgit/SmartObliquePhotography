#include "pch.h"
#include "CompositeBinaryClassifierAlg.h"
#include "BinaryClassifierAlg.h"
#include "PointCluster4VFH.h"
#include "PointCluster4Score.h"
#include "PointCluster4NormalRatio.h"

using namespace hiveObliquePhotography::AutoRetouch;

//*****************************************************************
//FUNCTION: 
void CCompositeBinaryClassifierAlg::addBinaryClassifiers(const std::vector<std::string>& vClusterTypes)
{
	_ASSERTE(!vClusterTypes.empty());

	m_ClusterTypes = vClusterTypes;
	for (auto& Type : vClusterTypes)
	{
		if (Type.find(BINARY_CLUSTER_VFH) != std::string::npos || Type.find(BINARY_CLUSTER_SCORE) != std::string::npos || Type.find(BINARY_CLUSTER_NORMAL) != std::string::npos)
		{
			CBinaryClassifierAlg* pClassifier = hiveDesignPattern::hiveCreateProduct<CBinaryClassifierAlg>(CLASSIFIER_BINARY, CPointCloudAutoRetouchScene::getInstance()->fetchPointLabelSet());
			_HIVE_EARLY_RETURN(!pClassifier, "Fail to execute binary classifier.");

			addClassifierAndExecute<CBinaryClassifierAlg>(pClassifier, Type);
		}
	}
}

//*****************************************************************
//FUNCTION: 
EPointLabel CCompositeBinaryClassifierAlg::__ensembleSingleResultV(const std::vector<SPointLabelChange>& vOverallResult) const
{
	_ASSERTE(m_ClusterTypes.size() == vOverallResult.size());

	float Score = 0.0f;
	const float VFH_WEIGHT = 0.5f;
	const float SCORE_WEIGHT = 0.3f;
	const float NORMAL_WEIGHT = 0.0f;

	for (int i = 0; i < vOverallResult.size(); i++)
	{
		if (vOverallResult[i].DstLabel == EPointLabel::UNWANTED)
		{
			auto Type = m_ClusterTypes[i];
			if (Type.find(BINARY_CLUSTER_VFH) == std::string::npos)
			{
				Score += 100.0f * VFH_WEIGHT * vOverallResult[i].Confidence;
			}
			else if (Type.find(BINARY_CLUSTER_SCORE) == std::string::npos)
			{
				Score += 100.0f * SCORE_WEIGHT * vOverallResult[i].Confidence;
			}
			else if (Type.find(BINARY_CLUSTER_NORMAL) == std::string::npos)
			{
				Score += 100.0f * NORMAL_WEIGHT * vOverallResult[i].Confidence;
			}
		}
	}

	if (Score > 90.0f)
		return EPointLabel::UNWANTED;
	else
		return EPointLabel::UNDETERMINED;
}
