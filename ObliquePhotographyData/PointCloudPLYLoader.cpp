#include "pch.h"
#include "PointCloudPLYLoader.h"

using namespace hiveObliquePhotography;

_REGISTER_EXCLUSIVE_PRODUCT(CPointCloudPLYLoader, PLY_LOADER)

//*****************************************************************
//FUNCTION:
int CPointCloudPLYLoader::__loadDataFromFileV(const std::string& vFileName, PointCloud_t::Ptr voPointCloud)
{
	return pcl::io::loadPLYFile<PointCloud_t::PointType>(vFileName, *voPointCloud);
}
