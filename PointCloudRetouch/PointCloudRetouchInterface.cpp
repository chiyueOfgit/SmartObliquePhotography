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
bool hiveObliquePhotography::PointCloudRetouch::hiveSave(PointCloud_t::Ptr voPointCloud)
{
	pcl::Indices PointIndices;
	CPointCloudRetouchManager::getInstance()->getIndicesByLabel(PointIndices, EPointLabel::KEPT);
	CPointCloudRetouchManager::getInstance()->getIndicesByLabel(PointIndices, EPointLabel::UNDETERMINED);
	for (auto Index : PointIndices)
	{
		pcl::PointSurfel TempPoint;
		auto Pos = CPointCloudRetouchManager::getInstance()->getRetouchScene().getPositionAt(Index);
		TempPoint.x = Pos.x();
		TempPoint.y = Pos.y();
		TempPoint.z = Pos.z();
		auto Normal = CPointCloudRetouchManager::getInstance()->getRetouchScene().getNormalAt(Index);
		TempPoint.normal_x = Normal.x();
		TempPoint.normal_y = Normal.y();
		TempPoint.normal_z = Normal.z();
		auto Color = CPointCloudRetouchManager::getInstance()->getRetouchScene().getColorAt(Index);
		TempPoint.r = Color.x();
		TempPoint.g = Color.y();
		TempPoint.b = Color.z();
		voPointCloud->push_back(TempPoint);
	}
	return true;
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::PointCloudRetouch::hiveMarkLitter(const std::vector<pcl::index_t>& vUserMarkedRegion, double vHardness, double vRadius, const Eigen::Vector2f& vCenter, const Eigen::Matrix4d& vPvMatrix, const std::pair<float, float>& vWindowSize)
{
	return CPointCloudRetouchManager::getInstance()->executeMarker(vUserMarkedRegion, vHardness,  vRadius, vCenter, vPvMatrix, vWindowSize, EPointLabel::UNWANTED);
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::PointCloudRetouch::hiveMarkBackground(const std::vector<pcl::index_t>& vUserMarkedRegion, double vHardness, double vRadius, const Eigen::Vector2f& vCenter, const Eigen::Matrix4d& vPvMatrix, const std::pair<float, float>& vWindowSize)
{
	return CPointCloudRetouchManager::getInstance()->executeMarker(vUserMarkedRegion, vHardness, vRadius, vCenter, vPvMatrix, vWindowSize, EPointLabel::KEPT);
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::PointCloudRetouch::hiveRemoveOutlier()
{
	return CPointCloudRetouchManager::getInstance()->executeRemoveOutlier();
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::PointCloudRetouch::hiveDumpPointLabel(std::vector<std::size_t>& voPointLabel)
{
	return CPointCloudRetouchManager::getInstance()->dumpPointLabel(voPointLabel);
}

void hiveObliquePhotography::PointCloudRetouch::hiveDiscardUnwantedPoints()
{
	CPointCloudRetouchManager::getInstance()->switchLabel(EPointLabel::DISCARDED, EPointLabel::UNWANTED);
}

void hiveObliquePhotography::PointCloudRetouch::hiveRecoverDiscardPoints2Unwanted()
{
	CPointCloudRetouchManager::getInstance()->switchLabel(EPointLabel::UNWANTED, EPointLabel::DISCARDED);
}

void hiveObliquePhotography::PointCloudRetouch::hiveClearMarkerResult()
{
	CPointCloudRetouchManager::getInstance()->clearMarkerResult();
}

void hiveObliquePhotography::PointCloudRetouch::hiveExecuteRubber(const std::vector<pcl::index_t>& vPoints)
{
	CPointCloudRetouchManager::getInstance()->setLabel(vPoints, EPointLabel::UNDETERMINED);
}
