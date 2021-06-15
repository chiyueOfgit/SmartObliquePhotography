#pragma once
#include "ObliquePhotographyDataExport.h"

using PointCloud_t = pcl::PointCloud<pcl::PointSurfel>;

namespace hiveObliquePhotography
{
	OPDATA_DECLSPEC PointCloud_t::Ptr hiveInitPointCloudScene(const std::vector<std::string>& vFileNameSet);
	OPDATA_DECLSPEC bool hiveSavePointCloudScene(PointCloud_t& vPointCloud, std::string vFileName);
}