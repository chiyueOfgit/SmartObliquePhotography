#pragma once
#include <common/HiveConfig.h>
#include <common/Singleton.h>
#include <common/ConfigInterface.h>

namespace hiveObliquePhotography
{
	namespace QTInterface
	{
		const std::string DOCKWIDGETTITLEBAR_BACKGROUNDCOLOR = "DOCKWIDGETTITLEBAR_BACKGROUNDCOLOR";
		const std::string DOCKWIDGETTITLEBAR_FONTCOLOR = "DOCKWIDGETTITLEBAR_FONTCOLOR";
		const std::string DOCKWIDGETTITLEBAR_FONTSIZE = "DOCKWIDGETTITLEBAR_FONTSIZE";
		const std::string SLIDER_SIZE = "SLIDER_SIZE";

		class CQInterfaceConfig : public hiveConfig::CHiveConfig, public hiveDesignPattern::CSingleton<CQInterfaceConfig>
		{
		public:
			~CQInterfaceConfig() override = default;

		private:
			CQInterfaceConfig()
			{
				CQInterfaceConfig::__defineAttributesV();

				std::string RelativePath = "../";
#ifdef _UNIT_TEST
				RelativePath = "../../";
#endif // _UNIT_TEST
				const std::string ConfigPath = "Configs/QTInterfaceConfig.xml";

				if (hiveConfig::hiveParseConfig(RelativePath + ConfigPath, hiveConfig::EConfigType::XML, this) != hiveConfig::EParseResult::SUCCEED)
				{
					_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
					return;
				}
			}

			void __defineAttributesV() override
			{
				_defineAttribute(DOCKWIDGETTITLEBAR_BACKGROUNDCOLOR, hiveConfig::EConfigDataType::ATTRIBUTE_VEC4I);
				_defineAttribute(DOCKWIDGETTITLEBAR_FONTCOLOR, hiveConfig::EConfigDataType::ATTRIBUTE_VEC4I);
				_defineAttribute(DOCKWIDGETTITLEBAR_FONTSIZE, hiveConfig::EConfigDataType::ATTRIBUTE_INT);
				_defineAttribute(SLIDER_SIZE, hiveConfig::EConfigDataType::ATTRIBUTE_VEC2I);
			}

			friend class hiveDesignPattern::CSingleton<CQInterfaceConfig>;
		};
	}
}
