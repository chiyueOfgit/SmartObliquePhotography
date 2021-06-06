#include "pch.h"
#include "PointCloudPCDLoader.h"
#include <boost/format.hpp>
#include <pcl/io/pcd_io.h>

using namespace hiveObliquePhotography;

//_REGISTER_NORMAL_PRODUCT(CPointCloudPCDLoader, PCD_LOADER)

//*****************************************************************
//FUNCTION: 
bool CPointCloudPCDLoader::__loadDataFromFileV(const std::string& vFileName, pcl::PointCloud<pcl::PointSurfel>& voPointCloud)
{
	if (pcl::io::loadPCDFile<pcl::PointSurfel>(vFileName, voPointCloud) == -1)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to load file [%1%] .", vFileName));
		return false;
	}
	return true;
}
