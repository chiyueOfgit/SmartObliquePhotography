#include "pch.h"
#include "PointLabelSet.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
void CPointLabelSet::tagCoreRegion4Cluster(const std::vector<pcl::index_t>& vTargetPointSet, EPointLabel vTargetLabel, std::uint32_t vClusterIndex)
{
	for (auto e : vTargetPointSet)
	{
		m_LabelSet[e].ClusterIndex = vClusterIndex;
		m_LabelSet[e].PointLabel   = vTargetLabel;
		m_LabelSet[e].Probability  = 1.0;
	}
}

//*****************************************************************
//FUNCTION: 
void CPointLabelSet::init(std::size_t vSize)
{
	_ASSERTE(m_LabelSet.empty() && (vSize > 0));
	m_LabelSet = std::vector<SPointLabel>(vSize);
}

//*****************************************************************
//FUNCTION: 
void CPointLabelSet::tagPointLabel(pcl::index_t vPoint, EPointLabel vTargetLabel, std::uint32_t vClusterIndex, double vClusterBelongingProbability)
{
	_ASSERTE(vPoint < m_LabelSet.size());
	m_LabelSet[vPoint].PointLabel = vTargetLabel;
	m_LabelSet[vPoint].ClusterIndex = vClusterIndex;
	m_LabelSet[vPoint].Probability = vClusterBelongingProbability;
}