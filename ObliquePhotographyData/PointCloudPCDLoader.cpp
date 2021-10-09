#include "pch.h"
#include "PointCloudPCDLoader.h"

using namespace hiveObliquePhotography;

_REGISTER_EXCLUSIVE_PRODUCT(CPointCloudPCDLoader, PCD_LOADER)

//*****************************************************************
//FUNCTION: 
int CPointCloudPCDLoader::__loadDataFromFileV(const std::string& vFileName, PointCloud_t::Ptr voPointCloud)
{
	return pcl::io::loadPCDFile<PointCloud_t::PointType>(vFileName, *voPointCloud);
}
