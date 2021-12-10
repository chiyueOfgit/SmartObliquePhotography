#include "pch.h"
#include "SmartOPInterface.h"
#include "OverallConfig.h"

using namespace hiveObliquePhotography;

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::_hiveParseMetafile(const std::string& vMetaFileName)
{
	_HIVE_EARLY_RETURN(!hiveUtility::hiveFileSystem::hiveIsFileExisted(vMetaFileName), _FORMAT_STR1("The specified metafile [%1%] does not exist.", vMetaFileName), false);

	try
	{
		_HIVE_EARLY_RETURN((hiveConfig::hiveParseConfig(vMetaFileName, hiveConfig::EConfigType::XML, COverallConfig::getInstance()) != hiveConfig::EParseResult::SUCCEED),
			_FORMAT_STR1("Fail to parse the specified metafile [%1%].", vMetaFileName), false);
		_HIVE_EARLY_RETURN(!COverallConfig::getInstance()->generateSceneInfo(),
			_FORMAT_STR1("The information provided by the specified metafile [%1%] is not enough to generate scene information.", vMetaFileName), false);
		return true;
	}
	catch (...)
	{
		_HIVE_OUTPUT_WARNING("Fail to execute _hiveParseMetafile() due to unexpected error");
		return true;
	}
}

