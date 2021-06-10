#pragma once
#include <common/HiveConfig.h>
#include <common/Singleton.h>

class CAutoRetouchConfig : public hiveConfig::CHiveConfig, hiveDesignPattern::CSingleton<CAutoRetouchConfig>
{
public:
	~CAutoRetouchConfig() override = default;

private:
	CAutoRetouchConfig() { CAutoRetouchConfig::__defineAttributesV(); }
	
	void __defineAttributesV() override
	{
		_defineAttribute("ENABLE_COLOR_TEST", hiveConfig::EConfigDataType::ATTRIBUTE_BOOL);
	}
	
	friend class hiveDesignPattern::CSingleton<CAutoRetouchConfig>;
};