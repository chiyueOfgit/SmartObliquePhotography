#include "pch.h"
#include "PointCloudRetouchInterface.h"
#include "PointCloudRetouchManager.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::PointCloudRetouch::hiveInit(PointCloud_t::Ptr vPointCloud, const hiveConfig::CHiveConfig* vConfig)
{
	_ASSERTE(vPointCloud && vConfig);
	return CPointCloudRetouchManager::getInstance()->init(vPointCloud, vConfig);
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::PointCloudRetouch::hiveMarkLitter(const std::vector<pcl::index_t>& vUserMarkedRegion, double vHardness, double vRadius, const Eigen::Vector2f& vCenter, const Eigen::Matrix4f& vPvMatrix, const std::pair<float, float>& vWindowSize)
{
	return CPointCloudRetouchManager::getInstance()->executeMarker(vUserMarkedRegion, vHardness,  vRadius, vCenter, vPvMatrix, vWindowSize, EPointLabel::UNWANTED);
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::PointCloudRetouch::hiveMarkBackground(const std::vector<pcl::index_t>& vUserMarkedRegion, double vHardness, double vRadius, const Eigen::Vector2f& vCenter, const Eigen::Matrix4f& vPvMatrix, const std::pair<float, float>& vWindowSize)
{
	return CPointCloudRetouchManager::getInstance()->executeMarker(vUserMarkedRegion, vHardness, vRadius, vCenter, vPvMatrix, vWindowSize, EPointLabel::KEPT);
}