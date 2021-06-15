#pragma once
#include <common/HiveConfig.h>
#include <common/Singleton.h>
#include <common/ConfigInterface.h>
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
			CAutoRetouchConfig()
			{ 
				CAutoRetouchConfig::__defineAttributesV();

				std::string RelativePath = "../";
#ifdef _UNIT_TEST
				RelativePath = "../../";
#endif // _UNIT_TEST

				const std::string ConfigPath = "Configs/AutoRetouchConfig.xml";
				if (hiveConfig::hiveParseConfig(RelativePath + ConfigPath, hiveConfig::EConfigType::XML, this) != hiveConfig::EParseResult::SUCCEED)
				{
					_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
					return;
				}
			}

			void __defineAttributesV() override
			{
				_defineAttribute(KEY_WORDS::ENABLE_COLOR_TEST, hiveConfig::EConfigDataType::ATTRIBUTE_BOOL);
				_defineAttribute(KEY_WORDS::COLOR_TEST_MODE, hiveConfig::EConfigDataType::ATTRIBUTE_INT);
				_defineAttribute(KEY_WORDS::COLOR_TEST_THRESHOLD, hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
				_defineAttribute(KEY_WORDS::ENABLE_GROUND_TEST, hiveConfig::EConfigDataType::ATTRIBUTE_BOOL);
				_defineAttribute(KEY_WORDS::GROUND_TEST_THRESHOLD, hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
				_defineAttribute(KEY_WORDS::ENABLE_NORMAL_TEST, hiveConfig::EConfigDataType::ATTRIBUTE_BOOL);
				_defineAttribute(KEY_WORDS::SEARCH_RADIUS, hiveConfig::EConfigDataType::ATTRIBUTE_DOUBLE);
				_defineAttribute(KEY_WORDS::POINT_SHOW_SIZE, hiveConfig::EConfigDataType::ATTRIBUTE_DOUBLE);
				_defineAttribute(KEY_WORDS::OUTLIER_MEAN_KNN_NUMBER, hiveConfig::EConfigDataType::ATTRIBUTE_INT);
				_defineAttribute(KEY_WORDS::OUTLIER_STD_MULTIPLE_THRESHOLD, hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
				_defineAttribute(KEY_WORDS::RESOLUTION, hiveConfig::EConfigDataType::ATTRIBUTE_INT);
				_defineAttribute(KEY_WORDS::CLUSTERTOLERANCE, hiveConfig::EConfigDataType::ATTRIBUTE_DOUBLE);
				_defineAttribute(KEY_WORDS::MINCLUSTERSIZE, hiveConfig::EConfigDataType::ATTRIBUTE_INT);
				_defineAttribute(KEY_WORDS::MAXCLUSTERSIZE, hiveConfig::EConfigDataType::ATTRIBUTE_INT);
				_defineAttribute(KEY_WORDS::EXCUTEAREA_EXPAND_RATIO, hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
				_defineAttribute(KEY_WORDS::BINARY_CLASSIFIER_NORMAL_RATIO_THRESHOLD, hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
				//composite binary
				_defineAttribute(KEY_WORDS::ENABLE_VFH, hiveConfig::EConfigDataType::ATTRIBUTE_BOOL);
				_defineAttribute(KEY_WORDS::ENABLE_SCORE, hiveConfig::EConfigDataType::ATTRIBUTE_BOOL);
				_defineAttribute(KEY_WORDS::ENABLE_NORMAL, hiveConfig::EConfigDataType::ATTRIBUTE_BOOL);
				_defineAttribute(KEY_WORDS::VFH_WEIGHT, hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
				_defineAttribute(KEY_WORDS::SCORE_WEIGHT, hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
				_defineAttribute(KEY_WORDS::NORMAL_WEIGHT, hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);
				_defineAttribute(KEY_WORDS::EXPECT_SCORE, hiveConfig::EConfigDataType::ATTRIBUTE_FLOAT);

			}

			friend class hiveDesignPattern::CSingleton<CAutoRetouchConfig>;
		};
	}
}
