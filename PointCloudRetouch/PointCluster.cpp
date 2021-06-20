#include "pch.h"
#include "PointCluster.h"
#include "Feature.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
double CPointCluster::evaluateProbability(pcl::index_t vInputPoint, std::uint32_t vCurrentTimestamp)
{
	_ASSERTE(!m_FeatureSet.empty() && !m_FeatureWeightSet.empty() && (vCurrentTimestamp >= m_CreationTimestamp));

	double Probability = 0;

	for (auto i=0; i<m_FeatureSet.size(); i++)
	{
		if (m_FeatureWeightSet[i] == 0) continue;

	}

	_ASSERTE((Probability >= 0) && (Probability <= 1));
	return Probability;
}

//*****************************************************************
//FUNCTION: 
bool CPointCluster::init(const hiveConfig::CHiveConfig *vConfig, std::uint32_t vClusterCenter, const std::vector<pcl::index_t>& vFeatureGenerationSet, const std::vector<pcl::index_t>& vValidationSet, std::uint32_t vCreationTimestamp)
{
	_ASSERTE(vConfig && !vFeatureGenerationSet.empty() && !vValidationSet.empty());
	m_pConfig = vConfig;
	m_CreationTimestamp = vCreationTimestamp;
	m_ClusterCenter = vClusterCenter;
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

