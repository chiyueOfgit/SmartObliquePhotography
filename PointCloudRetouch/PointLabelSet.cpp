#include "pch.h"
#include "PointLabelSet.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
void CPointLabelSet::tagCoreRegion4Cluster(const std::vector<pcl::index_t>& vTargetPointSet, EPointLabel vTargetLabel, std::uint32_t vClusterIndex)
{
	for (auto e : vTargetPointSet)
	{
		__throwLabelIndexOutOfRange(e);
		
		m_LabelSet[e].ClusterIndex = vClusterIndex;
		m_LabelSet[e].PointLabel   = vTargetLabel;
		m_LabelSet[e].Probability  = 1.0;
	}
}

//*****************************************************************
//FUNCTION: 
void CPointLabelSet::init(std::size_t vSize)
{
	_ASSERTE(vSize > 0);
	m_LabelSet = std::vector<SPointLabel>(vSize);
}

void CPointLabelSet::reset()
{
	m_LabelSet = std::vector<SPointLabel>(m_LabelSet.size());
}

//*****************************************************************
//FUNCTION: 
void CPointLabelSet::tagPointLabel(pcl::index_t vPoint, EPointLabel vTargetLabel, std::uint32_t vClusterIndex, double vClusterBelongingProbability)
{
	__throwLabelIndexOutOfRange(vPoint);
	if (vClusterBelongingProbability < 0)
		_THROW_RUNTIME_ERROR("Illegal probability input");
	
	m_LabelSet[vPoint].PointLabel = vTargetLabel;
	m_LabelSet[vPoint].ClusterIndex = vClusterIndex;
	m_LabelSet[vPoint].Probability = vClusterBelongingProbability;
}