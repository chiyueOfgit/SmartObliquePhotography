#include "pch.h"
#include "PointCloudPLYSaver.h"

using namespace hiveObliquePhotography;

_REGISTER_EXCLUSIVE_PRODUCT(CPointCloudPLYSaver, PLY_SAVER);

//*****************************************************************
//FUNCTION: 
void CPointCloudPLYSaver::saveDataToFile(const PointCloud_t& vPointCloud, const std::string& vFilePath)
{
	if (pcl::io::savePLYFileBinary<PointCloud_t::PointType>(vFilePath, vPointCloud) == -1)
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to write file [%1%] due to inexistent file.", vFilePath));
}