#pragma once
#include "PointCloudRetouchExport.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		RETOUCH_DECLSPEC bool hiveInit(PointCloud_t::Ptr vPointCloud, const hiveConfig::CHiveConfig* vConfig);
		RETOUCH_DECLSPEC bool hiveSave(PointCloud_t::Ptr voPointCloud);
		RETOUCH_DECLSPEC bool hiveMarkBackground(const std::vector<pcl::index_t>& vUserMarkedRegion, double vHardness, double vRadius, const Eigen::Vector2f& vCenter, const Eigen::Matrix4d& vPvMatrix, const std::pair<float, float>& vWindowSize);
		RETOUCH_DECLSPEC bool hiveMarkLitter(const std::vector<pcl::index_t>& vUserMarkedRegion, double vHardness, double vRadius, const Eigen::Vector2f& vCenter, const Eigen::Matrix4d& vPvMatrix, const std::pair<float, float>& vWindowSize);
		RETOUCH_DECLSPEC bool hiveDumpPointLabel(std::vector<std::size_t>& voPointLabel);
		RETOUCH_DECLSPEC void hiveDiscardUnwantedPoints();
		RETOUCH_DECLSPEC void hiveRecoverDiscardPoints2Unwanted();
		RETOUCH_DECLSPEC void hiveClearMarkerResult();
		//DEBUG
		RETOUCH_DECLSPEC bool hiveDumpExpandResult();
		//DEBUG
		RETOUCH_DECLSPEC void hiveExecuteRubber(const std::vector<pcl::index_t>& vPoints);
	}
}