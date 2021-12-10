#include "pch.h"
#include "UserDefinedCategoryMap.h"

using namespace hiveObliquePhotography;

//*****************************************************************
//FUNCTION: 
bool CUserDefinedCategoryMap::load()
{
	_ASSERTE(COverallConfig::getInstance()->isReady());

	try
	{
		return true;
	}
	catch (std::runtime_error* e)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to execute [CUserDefinedCategoryMap::load()] due to the following error: [%1%].", e->what()));
		return false;
	}
	catch (...)
	{
		return false;
	}
}

//*****************************************************************
//FUNCTION: 按最近邻采样方式来获取category，如果用户指定的经纬度超出范围，则返回EPointCategory::Undefined
EPointCategory CUserDefinedCategoryMap::getCategory(float vLongitude, float vLatitude)
{
	return EPointCategory::UNDEFINED;
}