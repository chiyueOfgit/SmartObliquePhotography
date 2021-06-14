#pragma once
#include <common/HiveConfig.h>
#include <common/Singleton.h>
#include "ConfigCommon.h"

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
				_defineAttribute(KEY_WORDS::ENABLE_COLOR_TEST, hiveConfig::EConfigDataType::ATTRIBUTE_BOOL);
				_defineAttribute(KEY_WORDS::COLOR_TEST_MODE, hiveConfig::EConfigDataType::ATTRIBUTE_INT);
				_defineAttribute(KEY_WORDS::COLOR_TEST_THRESHOLD, hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);

				_defineAttribute(KEY_WORDS::ENABLE_GROUND_TEST, hiveConfig::EConfigDataType::ATTRIBUTE_BOOL);
				_defineAttribute(KEY_WORDS::GROUND_TEST_THRESHOLD, hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);

				_defineAttribute(KEY_WORDS::ENABLE_NORMAL_TEST, hiveConfig::EConfigDataType::ATTRIBUTE_BOOL);

				_defineAttribute(KEY_WORDS::SEARCH_RADIUS, hiveConfig::EConfigDataType::ATTRIBUTE_DOUBLE);

				_defineAttribute("POINT_SHOW_SIZE", hiveConfig::EConfigDataType::ATTRIBUTE_DOUBLE);

				_defineAttribute("OUTLIER_MEAN_KNN_NUMBER", hiveConfig::EConfigDataType::ATTRIBUTE_INT);
				_defineAttribute("OUTLIER_STD_MULTIPLE_THRESHOLD", hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
			}

			friend class hiveDesignPattern::CSingleton<CAutoRetouchConfig>;
		};
	}
}
