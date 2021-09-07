#include "pch.h"
#include "MeshLoader.h"

using namespace hiveObliquePhotography;

//*****************************************************************
//FUNCTION: 
bool IMeshLoader::loadDataFromFile(CMesh& voMesh, const std::string& vFileName)
{
	_ASSERTE(!vFileName.empty());

	std::string FileName = hiveUtility::hiveLocateFile(vFileName);
	//_HIVE_EARLY_RETURN(FileName.empty(), _FORMAT_STR1("Fail to load file [%1%] because it does not exist.", vFileName), nullptr);
	if (FileName.empty())
		return false;

	CMesh Mesh;

	try
	{
		int result = __loadDataFromFileV(vFileName, Mesh);
		if (result == 0)
		{
			hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("Succeed to load point cloud file [%1%].", vFileName));
			return true;
		}
		else
			return false;
	}
	catch (...)
	{
		return false;
	}
}
