#include "pch.h"
#include "PointCloudLoader.h"

using namespace hiveObliquePhotography;

//*****************************************************************
//FUNCTION: 
pcl::PointCloud<pcl::PointSurfel>* IPointCloudLoader::loadDataFromFile(const std::string& vFileName)
{
	_ASSERTE(!vFileName.empty());

	pcl::PointCloud<pcl::PointSurfel>* pPointCloud = new pcl::PointCloud<pcl::PointSurfel>;

	if (!__loadDataFromFileV(vFileName, *pPointCloud))
	{
		_SAFE_DELETE(pPointCloud);
	}
	return pPointCloud;
}