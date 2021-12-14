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
//FUNCTION: ������ڲ�����ʽ����ȡcategory������û�ָ���ľ�γ�ȳ�����Χ���򷵻�EPointCategory::Undefined
EPointCategory CUserDefinedCategoryMap::getCategory(float vLongitude, float vLatitude)
{
	return EPointCategory::UNDEFINED;
}