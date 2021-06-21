#include "pch.h"
#include "EuclideanNeighborhoodBuilder.h"
#include "PointCloudRetouchManager.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CEuclideanNeighborhoodBuilder, KEYWORD::EUCLIDEAN_NEIGHBOR_BUILDER)

//*****************************************************************
//FUNCTION: 
void CEuclideanNeighborhoodBuilder::__extraInitV()
{
	m_pCloud.reset(new PointCloud_t);
	m_pTree.reset(new pcl::search::KdTree<pcl::PointSurfel>);

	auto CloudSize = CPointCloudRetouchManager::getInstance()->getRetouchScene().getNumPoint();
	for (int i = 0; i < CloudSize; i++)
	{
		pcl::PointSurfel TempPoint;
		auto Position = CPointCloudRetouchManager::getInstance()->getRetouchScene().getPositionAt(i);
		TempPoint.x = Position.x();
		TempPoint.y = Position.y();
		TempPoint.z = Position.z();
		
		m_pCloud->push_back(TempPoint);
	}

	m_pTree->setInputCloud(m_pCloud);
}

void CEuclideanNeighborhoodBuilder::__buildNeighborhoodV(pcl::index_t vSeed, std::vector<pcl::index_t>& voNeighborhood)
{
	//config
	const std::string SEARCH_MODE = "NEAREST";
	const int NEAREST_N = 10;
	const float RADIUS = 0.5;

	std::vector<float> Distance;
	if (SEARCH_MODE == "NEAREST")
		m_pTree->nearestKSearch(m_pCloud->points[vSeed], NEAREST_N, voNeighborhood, Distance);
	else if (SEARCH_MODE == "RADIUS")
		m_pTree->radiusSearch(m_pCloud->points[vSeed], RADIUS, voNeighborhood, Distance);
}