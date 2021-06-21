#include "pch.h"
#include "PointCluster.h"
#include "Feature.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
double CPointCluster::evaluateProbability(pcl::index_t vInputPoint) const
{
	_ASSERTE(!m_FeatureSet.empty() && !m_FeatureWeightSet.empty());

	double Probability = 0;

	for (auto i = 0; i < m_FeatureSet.size(); i++)
	{
		if (m_FeatureWeightSet[i] == 0) continue;
		
		Probability += m_FeatureWeightSet[i] * m_FeatureSet[i]->evaluateFeatureMatchFactorV(vInputPoint);
	}

	_ASSERTE((Probability >= 0) && (Probability <= 1));
	return Probability;
}

//*****************************************************************
//FUNCTION: 
bool CPointCluster::init(const hiveConfig::CHiveConfig* vConfig, std::uint32_t vClusterCenter, EPointLabel vLabel, const std::vector<pcl::index_t>& vFeatureGenerationSet, const std::vector<pcl::index_t>& vValidationSet, std::uint32_t vCreationTimestamp)
{
	_ASSERTE(vConfig && !vFeatureGenerationSet.empty() && !vValidationSet.empty());
	m_pConfig = vConfig;
	m_CreationTimestamp = vCreationTimestamp;
	m_ClusterCenter = vClusterCenter;
	m_Label = vLabel;
	m_ClusterCoreRegion = vFeatureGenerationSet;  //·¢Éústd::vector¿½±´

	__createFeatureObjectSet();

	for (auto e : m_FeatureSet)
	{
		m_FeatureWeightSet.emplace_back(e->generateFeatureV(vFeatureGenerationSet, vValidationSet, m_ClusterCenter));
	}

	return true;
}

//*****************************************************************
//FUNCTION: 
void CPointCluster::__createFeatureObjectSet()
{

}

//*****************************************************************
//FUNCTION: 
bool CPointCluster::isBelongingTo(double vProbability) const
{
	int Num = 0;
	float ExpectRatio = 60.0f;
	for (auto pFeature : m_FeatureSet)
	{
		if (vProbability > 0)//pFeature->getExpectProbability())
			Num++;
	}
	if (Num / m_FeatureSet.size() >= ExpectRatio)
		return true;
	else
		return false;
}