#include "pch.h"
#include "PointCloudPLYLoader.h"

using namespace hiveObliquePhotography;

_REGISTER_EXCLUSIVE_PRODUCT(CPointCloudPLYLoader, PLY_LOADER)

//*****************************************************************
//FUNCTION:
int CPointCloudPLYLoader::__loadDataFromFileV(const std::string& vFileName, pcl::PointCloud<pcl::PointSurfel>& voPointCloud)
{
	return pcl::io::loadPLYFile<pcl::PointSurfel>(vFileName, voPointCloud);
}
