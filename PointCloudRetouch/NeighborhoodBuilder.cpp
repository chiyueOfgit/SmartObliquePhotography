#include "pch.h"
#include "NeighborhoodBuilder.h"
#include "PointLabelSet.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

INeighborhoodBuilder::~INeighborhoodBuilder()
{

}

//*****************************************************************
//FUNCTION: 
bool INeighborhoodBuilder::onProductCreatedV(const hiveConfig::CHiveConfig* vConfig, PointCloud_t::Ptr vPointCloudScene, const CPointLabelSet* vPointLabelSet)
{
	_ASSERTE(vPointCloudScene && vPointLabelSet);
	m_pPointCloudScene = vPointCloudScene;
	m_pPointLabelSet = vPointLabelSet;

	m_ClusterTag.resize(vPointCloudScene->size());
	reset();

	__extraInitV(vConfig);
	return true;
}

//*****************************************************************
//FUNCTION: 
std::vector<pcl::index_t> INeighborhoodBuilder::buildNeighborhood(pcl::index_t vSeed, std::uint32_t vSeedClusterIndex)
{
	if (!m_pPointCloudScene)
		_THROW_RUNTIME_ERROR("PointCloud pointer is uninitialized");

	if (vSeed <0 || vSeed >= m_pPointCloudScene->size())
		_THROW_RUNTIME_ERROR("Seed index is out of range");

	std::vector<pcl::index_t> Neighborhood;

	for (auto e : __buildNeighborhoodV(vSeed))
	{
		if (m_ClusterTag[e] != vSeedClusterIndex)
		{
			Neighborhood.push_back(e);
			m_ClusterTag[e] = vSeedClusterIndex;
		}
	}
	//·¢ÉúNRVO
	return Neighborhood;
}

//*****************************************************************
//FUNCTION: 
void INeighborhoodBuilder::reset()
{
	_ASSERTE(m_pVisitedTag);
	for (auto i = 0; i < m_pPointCloudScene->size(); i++) m_ClusterTag[i] = -1;
}
