#include "pch.h"
#include "PointCloudFileLoader.h"

using namespace hiveObliquePhotography;

//*****************************************************************
//FUNCTION: 
SUnorganizedPointCloud* IPointCloudFileLoader::load(const std::string& vFileName)
{
	try
	{
		_HIVE_EARLY_RETURN(!hiveUtility::hiveFileSystem::hiveIsFileExisted(vFileName),
			_FORMAT_STR1("Fail to load the specified point cloud file [%1%] because it does not exist.", vFileName), nullptr);

		return __loadV(vFileName);
	}
	catch (...)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to load point cloud file [%1%] due to unexpected error.", vFileName));
		return nullptr;
	}
}