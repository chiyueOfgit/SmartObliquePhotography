#pragma once
#include "PointCloudRetouchExport.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		RETOUCH_DECLSPEC bool hiveInit(PointCloud_t::Ptr vPointCloud, const hiveConfig::CHiveConfig* vConfig);
		RETOUCH_DECLSPEC bool hiveUndo();
		RETOUCH_DECLSPEC bool hiveMarkBackground(const std::vector<pcl::index_t>& vUserMarkedRegion, double vHardness, double vRadius, const Eigen::Vector2f& vCenter, const Eigen::Matrix4d& vPvMatrix, const std::pair<float, float>& vWindowSize);
		RETOUCH_DECLSPEC bool hiveMarkLitter(const std::vector<pcl::index_t>& vUserMarkedRegion, double vHardness, double vRadius, const Eigen::Vector2f& vCenter, const Eigen::Matrix4d& vPvMatrix, const std::pair<float, float>& vWindowSize);
		RETOUCH_DECLSPEC bool hiveMarkIsolatedAreaAsLitter();
		RETOUCH_DECLSPEC void hiveClearMark();
		RETOUCH_DECLSPEC void hiveDiscardLitter();
		RETOUCH_DECLSPEC void hiveRestoreLitter();

		RETOUCH_DECLSPEC bool hiveDumpPointCloudtoSave(PointCloud_t::Ptr voPointCloud);
		RETOUCH_DECLSPEC bool hiveDumpPointLabel(std::vector<std::size_t>& voPointLabel);
		RETOUCH_DECLSPEC void hiveDumpExpandResult(std::vector<pcl::index_t>& voExpandPoints, bool vIsLitterMarker = true);

		//TODO: È¨ÏÞÂÔ´ó
		RETOUCH_DECLSPEC void hiveEraseMark(const std::vector<pcl::index_t>& vPoints);
	}
}