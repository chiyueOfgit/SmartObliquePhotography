#pragma once
#include "PointCloudRetouchExport.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		RETOUCH_DECLSPEC bool hiveInit(PointCloud_t::Ptr vPointCloud, const std::vector<PointCloud_t::Ptr>& vTileSet, const hiveConfig::CHiveConfig* vConfig);
		RETOUCH_DECLSPEC bool hiveUndo();
		RETOUCH_DECLSPEC bool hivePreprocessSelected(std::vector<pcl::index_t>& vioSelected, const Eigen::Matrix4d& vPvMatrix, const std::function<double(Eigen::Vector2d)>& vSignedDistanceFunc, const Eigen::Vector3d& vViewPos);
		RETOUCH_DECLSPEC bool hiveMarkBackground(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix, const std::function<double(Eigen::Vector2d)>& vHardnessFunc);
		RETOUCH_DECLSPEC bool hiveMarkLitter(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix, const std::function<double(Eigen::Vector2d)>& vHardnessFunc);
		RETOUCH_DECLSPEC bool hiveMarkIsolatedAreaAsLitter();
		RETOUCH_DECLSPEC void hiveClearMark();
		RETOUCH_DECLSPEC void hiveRecoverLitterMark();
		RETOUCH_DECLSPEC void hiveRecoverBackgroundMark();
		RETOUCH_DECLSPEC void hiveHideLitter();
		RETOUCH_DECLSPEC void hiveDisplayLitter();
		RETOUCH_DECLSPEC void hiveRepairHoleSetRepairRegion(std::vector<pcl::index_t>& vRepairRegion);
		RETOUCH_DECLSPEC void hiveRepairHoleSetReferenceRegion(std::vector<pcl::index_t>& vReferenceRegion);
		RETOUCH_DECLSPEC void hiveRepairHole(std::vector<pcl::PointXYZRGBNormal>& voNewPoints);
		
		RETOUCH_DECLSPEC bool hiveDumpPointCloudtoSave(PointCloud_t::Ptr voPointCloud);
		RETOUCH_DECLSPEC bool hiveDumpPointLabel(std::vector<std::size_t>& voPointLabel);
		RETOUCH_DECLSPEC void hiveDumpExpandResult(std::vector<pcl::index_t>& voExpandPoints, bool vIsLitterMarker = true);

		RETOUCH_DECLSPEC bool hiveDumpColorFeatureMainColors(std::vector<Eigen::Vector3i>& voMainColors);
		RETOUCH_DECLSPEC bool hiveDumpColorFeatureNearestPoints(std::vector<pcl::index_t>& vNearestPoints);

		RETOUCH_DECLSPEC void hiveRunPrecompute(const std::string& vModelName);

		//TODO: È¨ÏÞÂÔ´ó
		RETOUCH_DECLSPEC void hiveEraseMark(const std::vector<pcl::index_t>& vPoints);
	}
}