#pragma once
#include <common/HiveConfig.h>
#include <common/Singleton.h>

class CAutoRetouchConfig final : public hiveConfig::CHiveConfig, hiveDesignPattern::CSingleton<CAutoRetouchConfig>
{
public:
	CAutoRetouchConfig() { CAutoRetouchConfig::__defineAttributesV(); }
	~CAutoRetouchConfig() override = default;

private:
	void __defineAttributesV() override
	{
		_defineAttribute("ENABLE_COLOR_TEST", hiveConfig::EConfigDataType::ATTRIBUTE_BOOL);
	}
	
	friend class hiveDesignPattern::CSingleton<CAutoRetouchConfig>;
};