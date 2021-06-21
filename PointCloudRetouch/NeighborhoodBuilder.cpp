#include "pch.h"
#include "NeighborhoodBuilder.h"
#include "PointLabelSet.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

INeighborhoodBuilder::~INeighborhoodBuilder()
{
	delete[] m_pVisitedTag;
}

//*****************************************************************
//FUNCTION: 
bool INeighborhoodBuilder::onProductCreatedV(PointCloud_t::Ptr vPointCloudScene, const CPointLabelSet* vPointLabelSet)
{
	_ASSERTE(vPointCloudScene && vPointLabelSet);
	m_pPointCloudScene = vPointCloudScene;
	m_pPointLabelSet = vPointLabelSet;

	m_pVisitedTag = new bool[m_pPointCloudScene->size()];
	reset();

	__extraInitV();
	return true;
}

//*****************************************************************
//FUNCTION: 
void INeighborhoodBuilder::buildNeighborhood(pcl::index_t vSeed, std::uint32_t vSeedClusterIndex, std::vector<pcl::index_t>& voNeighborhood)
{
	_ASSERTE(m_pPointCloudScene && (vSeed < m_pPointCloudScene->size()));

	std::vector<pcl::index_t> Neighborhood;
	__buildNeighborhoodV(vSeed, Neighborhood);

	voNeighborhood.clear();
	for (auto e : Neighborhood)
	{
		if ((m_pPointLabelSet->getClusterIndexAt(e) != vSeedClusterIndex) && !m_pVisitedTag[e])
		{
			voNeighborhood.emplace_back(e);
			m_pVisitedTag[e] = true;
		}
	}
}

//*****************************************************************
//FUNCTION: 
void INeighborhoodBuilder::reset()
{
	_ASSERTE(m_pVisitedTag);
	for (auto i = 0; i < m_pPointCloudScene->size(); i++) m_pVisitedTag[i] = false;
}
