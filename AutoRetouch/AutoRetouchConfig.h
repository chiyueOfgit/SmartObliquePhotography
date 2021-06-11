#pragma once
#include <common/HiveConfig.h>
#include <common/Singleton.h>

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class CAutoRetouchConfig : public hiveConfig::CHiveConfig, public hiveDesignPattern::CSingleton<CAutoRetouchConfig>
		{
		public:
			~CAutoRetouchConfig() override = default;

		private:
			CAutoRetouchConfig() { CAutoRetouchConfig::__defineAttributesV(); }

			void __defineAttributesV() override
			{
				_defineAttribute("ENABLE_COLOR_TEST", hiveConfig::EConfigDataType::ATTRIBUTE_BOOL);
				_defineAttribute("COLOR_TEST_MODE", hiveConfig::EConfigDataType::ATTRIBUTE_INT);
				_defineAttribute("COLOR_TEST_THRESHOLD", hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);

				_defineAttribute("ENABLE_GROUND_TEST", hiveConfig::EConfigDataType::ATTRIBUTE_BOOL);
				_defineAttribute("GROUND_TEST_THRESHOLD", hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);

				_defineAttribute("ENABLE_NORMAL_TEST", hiveConfig::EConfigDataType::ATTRIBUTE_BOOL);

				_defineAttribute("SEARCH_RADIUS", hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
			}

			friend class hiveDesignPattern::CSingleton<CAutoRetouchConfig>;
		};
	}
}
