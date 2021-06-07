#include "pch.h"
#include "PointCloudLoader.h"
#include "PointCloudPCDLoader.h"
#include "PointCloudPLYLoader.h"

using namespace hiveObliquePhotography;

_REGISTER_EXCLUSIVE_PRODUCT(CPointCloudPCDLoader, PCD_LOADER)
_REGISTER_EXCLUSIVE_PRODUCT(CPointCloudPLYLoader, PLY_LOADER)

//*****************************************************************
//FUNCTION: 
pcl::PointCloud<pcl::PointSurfel>* IPointCloudLoader::loadDataFromFile(const std::string& vFileName)
{
	_ASSERTE(!vFileName.empty());

	auto* pPointCloud = new pcl::PointCloud<pcl::PointSurfel>;

	if (!__loadDataFromFileV(vFileName, *pPointCloud))
	{
		_SAFE_DELETE(pPointCloud);
	}
	return pPointCloud;
}