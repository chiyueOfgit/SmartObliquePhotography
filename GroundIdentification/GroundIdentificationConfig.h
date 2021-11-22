#pragma once
#include "GroundIdentificationCommon.h"
#include <common/HiveConfig.h>
#include <common/Singleton.h>
#include <common/ConfigInterface.h>

namespace hiveObliquePhotography
{
	namespace GroundIdentification
	{
		class CGroundIdentificationConfig : public hiveConfig::CHiveConfig, public hiveDesignPattern::CSingleton<CGroundIdentificationConfig>
		{
		public:
			~CGroundIdentificationConfig() override = default;

			const hiveConfig::CHiveConfig* getSubConfigByName(const std::string& vName) { return findSubconfigByName(vName); }

		private:
			CGroundIdentificationConfig()
			{
				CGroundIdentificationConfig::__defineAttributesV();

				const std::string ConfigPath = "Config/GroundIdentificationConfig.xml";
				if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, this) != hiveConfig::EParseResult::SUCCEED)
				{
					_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
					return;
				}
			}

			void __defineAttributesV() override
			{
				
			}

			friend class hiveDesignPattern::CSingleton<CGroundIdentificationConfig>;
		};
	}
}