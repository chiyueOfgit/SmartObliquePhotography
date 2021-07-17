#include "pch.h"
#include "OutlierDetector.h"
#include "PointCloudRetouchManager.h"
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>


using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(COutlierDetector, KEYWORD::OUTLIER_DETECTOR)

//*****************************************************************
//FUNCTION: 
void COutlierDetector::runV(pcl::Indices& vInputIndices, EPointLabel vExpectLabel, const hiveConfig::CHiveConfig* vConfig)
{
	if (vInputIndices.empty())
		return;
	auto pManager = CPointCloudRetouchManager::getInstance();
	for (auto CurrentIndex : vInputIndices)
		if (CurrentIndex < 0 || CurrentIndex >= pManager->getRetouchScene().getNumPoint())
			_THROW_RUNTIME_ERROR("Index is out of range");
	
	PointCloud_t::Ptr pCloud(new pcl::PointCloud<pcl::PointSurfel>);
	for (auto Index : vInputIndices)
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
		TempPoint.curvature = Index;
		pCloud->push_back(TempPoint);
	}
	PointCloud_t::Ptr pResultCloud(new pcl::PointCloud<pcl::PointSurfel>);
	pcl::RadiusOutlierRemoval<pcl::PointSurfel> RadiusOutlier;
	RadiusOutlier.setInputCloud(pCloud);
	RadiusOutlier.setRadiusSearch(vConfig->getAttribute<float>("SEARCH_RADIUS").value());
	RadiusOutlier.setMinNeighborsInRadius(vConfig->getAttribute<int>("MIN_NEIGHBORS_IN_RADIUS").value());
	RadiusOutlier.setNegative(vConfig->getAttribute<bool>("POINT_FILTER_CONDITION").value());
	RadiusOutlier.filter(*pResultCloud);

	for (auto& Point : pResultCloud->points)
	   pManager->tagPointLabel(Point.curvature, vExpectLabel, 0, 0);
	
	pManager->recordCurrentStatus();
}

