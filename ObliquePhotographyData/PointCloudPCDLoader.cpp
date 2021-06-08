#include "pch.h"
#include "PointCloudPCDLoader.h"
#include <boost/format.hpp>
#include <pcl/io/pcd_io.h>

using namespace hiveObliquePhotography;

//_REGISTER_EXCLUSIVE_PRODUCT(CPointCloudPCDLoader, PCD_LOADER)

//*****************************************************************
//FUNCTION: 
bool CPointCloudPCDLoader::__loadDataFromFileV(const std::string& vFileName, PointCloud_t& voPointCloud)
{
	if (pcl::io::loadPCDFile<PointCloud_t::PointType>(vFileName, voPointCloud) < 0)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to load file [%1%] due to inexistent file.", vFileName));
		return false;
	}
	return true;
}
