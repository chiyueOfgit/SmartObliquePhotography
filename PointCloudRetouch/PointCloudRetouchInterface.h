#pragma once
#include "PointCloudRetouchExport.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		RETOUCH_DECLSPEC bool hiveInit(PointCloud_t::Ptr vPointCloud, const hiveConfig::CHiveConfig* vConfig);
		RETOUCH_DECLSPEC bool hiveMarkBackground(const std::vector<pcl::index_t>& vUserMarkedRegion, double vHardness, double vRadius, const Eigen::Vector3f& vCameraPos, const Eigen::Matrix4d& vPvMatrix);
		RETOUCH_DECLSPEC bool hiveMarkLitter(const std::vector<pcl::index_t>& vUserMarkedRegion, double vHardness, double vRadius, const Eigen::Vector3f& vCameraPos, const Eigen::Matrix4d& vPvMatrix);
	}
}