#pragma once
#include <common/HiveConfig.h>
#include <common/Singleton.h>
#include <common/ConfigInterface.h>

const std::string STEP_RATIO = "STEP_RATIO";

class CSingleStepConfig : public hiveConfig::CHiveConfig, public hiveDesignPattern::CSingleton<CSingleStepConfig>
{
public:
	~CSingleStepConfig() override = default;

private:
	CSingleStepConfig()
	{
		CSingleStepConfig::__defineAttributesV();

		const std::string ConfigPath = "SingleStepConfig.xml";

		if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, this) != hiveConfig::EParseResult::SUCCEED)
		{
			_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
			return;
		}
	}

	void __defineAttributesV() override
	{
		_defineAttribute(STEP_RATIO, hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);

	}

	friend class hiveDesignPattern::CSingleton<CSingleStepConfig>;
};