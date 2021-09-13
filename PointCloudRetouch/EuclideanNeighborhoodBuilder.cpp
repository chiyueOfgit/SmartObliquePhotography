#include "pch.h"
#include "EuclideanNeighborhoodBuilder.h"
#include "PointCloudRetouchManager.h"   //FIXME-014: 用到这个头文件了吗？

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
std::vector<pcl::index_t> CEuclideanNeighborhoodBuilder::__buildNeighborhoodV(pcl::index_t vSeed, std::string& vType, float vPara) const  //FIXME-014：两个_buildNeighborhoodV()函数copy/paste的吧，有必要做两个虚函数吗？
{//FIXME-014：输入参数vType为什么不加const修饰，你要改它吗？
//FIXME-014：为什么vType要用字符串？大小写敏感吗？传入了非法字符串怎么办？
//FIXME-014：为什么不对m_pTree做有效性检查？
//FIXME-014: 为什么在基类采用了工厂模式后，这里还要vType这个参数？
	std::vector<pcl::index_t> Neighborhood;
	std::vector<float> Distance;
	if (vType == "NEAREST")
		m_pTree->nearestKSearch(m_pPointCloudScene->points[vSeed], static_cast<int>(vPara), Neighborhood, Distance);
	else if (vType == "RADIUS")
		m_pTree->radiusSearch(m_pPointCloudScene->points[vSeed], vPara, Neighborhood, Distance);
	//发生NRVO
	return Neighborhood;
}

std::vector<pcl::index_t> CEuclideanNeighborhoodBuilder::__buildNeighborhoodV(pcl::index_t vSeed) const
{
	std::vector<pcl::index_t> Neighborhood;
	std::vector<float> Distance;
	if (m_SearchMode == "NEAREST")
		m_pTree->nearestKSearch(m_pPointCloudScene->points[vSeed], m_NearestN, Neighborhood, Distance);
	else if (m_SearchMode == "RADIUS")
		m_pTree->radiusSearch(m_pPointCloudScene->points[vSeed], m_Radius, Neighborhood, Distance);
	//发生NRVO
	return Neighborhood;
}