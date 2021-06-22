#include "pch.h"
#include "EuclideanNeighborhoodBuilder.h"
#include "PointCloudRetouchManager.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_NORMAL_PRODUCT(CEuclideanNeighborhoodBuilder, KEYWORD::EUCLIDEAN_NEIGHBOR_BUILDER)

//*****************************************************************
//FUNCTION: 
void CEuclideanNeighborhoodBuilder::__extraInitV(const hiveConfig::CHiveConfig* vConfig)
{
	m_pTree.reset(new pcl::search::KdTree<pcl::PointSurfel>);
	m_pTree->setInputCloud(m_pPointCloudScene);

	m_SearchMode = *vConfig->getAttribute<std::string>("SEARCH_MODE");
	m_NearestN = *vConfig->getAttribute<int>("NEAREST_N");
	m_Radius = *vConfig->getAttribute<float>("RADIUS");
}

//*****************************************************************
//FUNCTION: 
std::vector<pcl::index_t> CEuclideanNeighborhoodBuilder::__buildNeighborhoodV(pcl::index_t vSeed)
{
	std::vector<pcl::index_t> Neighborhood;
	std::vector<float> Distance;
	if (m_SearchMode == "NEAREST")
		m_pTree->nearestKSearch(m_pPointCloudScene->points[vSeed], m_NearestN, Neighborhood, Distance);
	else if (m_SearchMode == "RADIUS")
		m_pTree->radiusSearch(m_pPointCloudScene->points[vSeed], m_Radius, Neighborhood, Distance);
	//·¢ÉúNRVO
	return Neighborhood;
}