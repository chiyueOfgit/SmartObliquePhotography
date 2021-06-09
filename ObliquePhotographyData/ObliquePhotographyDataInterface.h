#pragma once
#include "ObliquePhotographyDataExport.h"

namespace hiveObliquePhotography
{
	OPDATA_DECLSPEC pcl::PointCloud<pcl::PointSurfel>::Ptr hiveInitPointCloudScene(const std::vector<std::string>& vFileNameSet);

}