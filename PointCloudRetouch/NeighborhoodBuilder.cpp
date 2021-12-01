#include "pch.h"
#include "NeighborhoodBuilder.h"
#include "PointLabelSet.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

INeighborhoodBuilder::~INeighborhoodBuilder()
{
	delete m_pPointLabelSet;
}

//*****************************************************************
//FUNCTION: 
bool INeighborhoodBuilder::onProductCreatedV(const hiveConfig::CHiveConfig* vConfig, const std::vector<PointCloud_t::Ptr>& vTileSet, const CPointLabelSet* vPointLabelSet)
{
	_ASSERTE(!vTileSet.empty() && vPointLabelSet);
	m_TileSet = vTileSet;
	m_pPointLabelSet = vPointLabelSet;
	m_NumPoints = 0;
	for (int i = 0, Offset = 0; i < m_TileSet.size(); i++)
	{
		m_OffsetSet.push_back(Offset);
		Offset += m_TileSet[i]->size();
		m_NumPoints += m_TileSet[i]->size();
	}

	//reset();

	__extraInitV(vConfig);
	return true;
}

//*****************************************************************
//FUNCTION: 
std::vector<pcl::index_t> INeighborhoodBuilder::buildNeighborhood(pcl::index_t vSeed, std::string& vType, float vPara) const
{
	if (m_TileSet.empty())
		_THROW_RUNTIME_ERROR("PointCloud pointer is uninitialized");

	if (vSeed <0 || vSeed >= m_NumPoints)
		_THROW_RUNTIME_ERROR("Seed index is out of range");

	std::vector<pcl::index_t> Neighborhood;

	for (auto e : __buildNeighborhoodV(vSeed, vType, vPara))
	{
		Neighborhood.push_back(e);
	}
	//发生NRVO
	return Neighborhood;
}

//*****************************************************************
//FUNCTION: 
std::vector<pcl::index_t> INeighborhoodBuilder::buildNeighborhood(pcl::index_t vSeed) const  //FIXME-014：两个buildNeighborhood()函数copy/paste的吧，改不了这个习惯吗？
{
	if (m_TileSet.empty())
		_THROW_RUNTIME_ERROR("PointCloud pointer is uninitialized");

	if (vSeed < 0 || vSeed >= m_NumPoints)
		_THROW_RUNTIME_ERROR("Seed index is out of range");

	std::vector<pcl::index_t> Neighborhood;

	for (auto e : __buildNeighborhoodV(vSeed))
	{
		Neighborhood.push_back(e);
	}
	//发生NRVO
	return Neighborhood;
}
