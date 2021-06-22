#include "pch.h"
#include "PointCloudLoader.h"

using namespace hiveObliquePhotography;

//*****************************************************************
//FUNCTION: 
PointCloud_t::Ptr IPointCloudLoader::loadDataFromFile(const std::string& vFileName)
{
	_ASSERTE(!vFileName.empty());

	std::string FileName = hiveUtility::hiveLocateFile(vFileName);
	_HIVE_EARLY_RETURN(FileName.empty(), _FORMAT_STR1("Fail to load file [%1%] because it does not exist.", vFileName), nullptr);

	PointCloud_t::Ptr pPointCloud(new PointCloud_t);

	try
	{
		int r = __loadDataFromFileV(vFileName, *pPointCloud);
		if (r == 0)
		{
			hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("Succeed to load point cloud file [%1%].", vFileName));
			return pPointCloud;
		}
		else
		{
			pPointCloud.reset();
			_HIVE_OUTPUT_WARNING(_FORMAT_STR2("Fail to load file [%1%] due to error [%2%].", vFileName, r));
			return nullptr;
		}
	}
	catch (...)
	{
		pPointCloud.reset();
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to load file [%1%] due to unexpeced error.", vFileName));
		return nullptr;
	}
}