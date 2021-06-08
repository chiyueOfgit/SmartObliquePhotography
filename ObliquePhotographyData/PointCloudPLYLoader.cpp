#include "pch.h"
#include "PointCloudPLYLoader.h"
#include <boost/format.hpp>
#include <pcl/io/ply_io.h>

using namespace hiveObliquePhotography;

//_REGISTER_EXCLUSIVE_PRODUCT(CPointCloudPLYLoader, PLY_LOADER)

//*****************************************************************
//FUNCTION:
bool CPointCloudPLYLoader::__loadDataFromFileV(const std::string& vFileName, PointCloud_t::Ptr voPointCloud)
{
	if (pcl::io::loadPLYFile<PointCloud_t::PointType>(vFileName, *voPointCloud) < 0)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to load file [%1%] due to inexistent file.", vFileName));
		return false;
	}
	return true;
}
