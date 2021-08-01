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

	reset();

	__extraInitV(vConfig);
	return true;
}

//*****************************************************************
//FUNCTION: 
std::vector<pcl::index_t> INeighborhoodBuilder::buildNeighborhood(pcl::index_t vSeed, std::string& vType, float vPara) const
{
	if (!m_pPointCloudScene)
		_THROW_RUNTIME_ERROR("PointCloud pointer is uninitialized");

	if (vSeed <0 || vSeed >= m_pPointCloudScene->size())
		_THROW_RUNTIME_ERROR("Seed index is out of range");

	std::vector<pcl::index_t> Neighborhood;

	for (auto e : __buildNeighborhoodV(vSeed, vType, vPara))
	{
		Neighborhood.push_back(e);
	}
	//发生NRVO
	return Neighborhood;
}

std::vector<pcl::index_t> INeighborhoodBuilder::buildNeighborhood(pcl::index_t vSeed) const
{
	if (!m_pPointCloudScene)
		_THROW_RUNTIME_ERROR("PointCloud pointer is uninitialized");

	if (vSeed < 0 || vSeed >= m_pPointCloudScene->size())
		_THROW_RUNTIME_ERROR("Seed index is out of range");

	std::vector<pcl::index_t> Neighborhood;

	for (auto e : __buildNeighborhoodV(vSeed))
	{
		Neighborhood.push_back(e);
	}
	//发生NRVO
	return Neighborhood;
}

//*****************************************************************
//FUNCTION: 
void INeighborhoodBuilder::reset()
{
}
