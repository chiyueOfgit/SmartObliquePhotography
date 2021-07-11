#pragma once
#include "PointCloudRetouchExport.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		RETOUCH_DECLSPEC bool hiveInit(PointCloud_t::Ptr vPointCloud, const hiveConfig::CHiveConfig* vConfig);
		RETOUCH_DECLSPEC bool hiveSave(PointCloud_t::Ptr voPointCloud);
		RETOUCH_DECLSPEC bool hivePreprocessSelected(std::vector<pcl::index_t>& vioSelected, const Eigen::Matrix4d& vPvMatrix, const std::function<float(Eigen::Vector2f)>& vSignedDistanceFunc, const Eigen::Vector3f& vViewPos);
		RETOUCH_DECLSPEC bool hiveMarkBackground(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix, double vHardness);
		RETOUCH_DECLSPEC bool hiveMarkLitter(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix, double vHardness);
		RETOUCH_DECLSPEC bool hiveRemoveOutlier();
		RETOUCH_DECLSPEC bool hiveDumpPointLabel(std::vector<std::size_t>& voPointLabel);
		RETOUCH_DECLSPEC void hiveDiscardUnwantedPoints();
		RETOUCH_DECLSPEC void hiveRecoverDiscardPoints2Unwanted();
		RETOUCH_DECLSPEC void hiveClearMarkerResult();

		RETOUCH_DECLSPEC void hiveDumpExpandResult(std::vector<pcl::index_t>& voExpandPoints);

		//DEBUG
		RETOUCH_DECLSPEC void hiveExecuteRubber(const std::vector<pcl::index_t>& vPoints);
	}
}