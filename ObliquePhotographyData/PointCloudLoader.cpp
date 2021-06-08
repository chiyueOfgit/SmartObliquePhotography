#include "pch.h"
#include "PointCloudLoader.h"
#include "PointCloudPCDLoader.h"
#include "PointCloudPLYLoader.h"

using namespace hiveObliquePhotography;

_REGISTER_EXCLUSIVE_PRODUCT(CPointCloudPCDLoader, PCD_LOADER)
_REGISTER_EXCLUSIVE_PRODUCT(CPointCloudPLYLoader, PLY_LOADER)

//*****************************************************************
//FUNCTION: 
PointCloud_t::Ptr IPointCloudLoader::loadDataFromFile(const std::string& vFileName)
{
	_ASSERTE(!vFileName.empty());

	PointCloud_t::Ptr pPointCloud(new PointCloud_t);
	
	if (!__loadDataFromFileV(vFileName, *pPointCloud))
		pPointCloud = nullptr;
	return pPointCloud;
}