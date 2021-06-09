#pragma once
#include "ObliquePhotographyDataExport.h"

namespace hiveObliquePhotography
{
	OPDATA_DECLSPEC pcl::PointCloud<pcl::PointSurfel>* hiveInitPointCloudScene(const std::vector<std::string>& vFileNameSet);
}