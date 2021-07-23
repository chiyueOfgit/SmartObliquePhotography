#pragma once
#include <common/HiveConfig.h>
#include <common/Singleton.h>
#include <common/ConfigInterface.h>

namespace hiveObliquePhotography
{
	namespace Visualization
	{
		const std::string SWITCH_UNWANTED_KEPT_MODE = "SWITCH_UNWANTED_KEPT_MODE";
		const std::string RECOVER_BACKGROUND_POINTS = "RECOVER_BACKGROUND_POINTS";
		const std::string SWITCH_UNWANTED_DISCARD = "SWITCH_UNWANTED_DISCARD";
		const std::string REMOVE_OUTLIER = "REMOVE_OUTLIER";
		const std::string UNDO = "UNDO";
		const std::string RADIUS_UP = "RADIUS_UP";
		const std::string RADIUS_DOWN = "RADIUS_DOWN";
		const std::string POINT_SIZE_UP = "POINT_SIZE_UP";
		const std::string POINT_SIZE_DOWN = "POINT_SIZE_DOWN";

		const std::string POINT_SHOW_SIZE = "POINT_SHOW_SIZE";
		const std::string SCREEN_CIRCLE_RADIUS = "SCREEN_CIRCLE_RADIUS";
		const std::string SCREEN_CIRCLE_HARDNESS = "SCREEN_CIRCLE_HARDNESS";

		const std::string CIRCLE_MODE = "CIRCLE_MODE";
		const std::string AREA_MODE = "AREA_MODE";
		const std::string AREA_PICK_CULLING = "AREA_PICK_CULLING";
		const std::string RUBBER_MODE = "RUBBER_MODE";
		const std::string UNWANTED_MODE = "UNWANTED_MODE";
		const std::string REFRESH_IMMEDIATELY = "REFRESH_IMMEDIATELY";

		const std::string LITTER_HIGHLIGHT_COLOR = "LITTER_HIGHLIGHT_COLOR";
		const std::string BACKGROUND_HIGHLIGHT_COLOR = "BACKGROUND_HIGHLIGHT_COLOR";

		class CVisualizationConfig : public hiveConfig::CHiveConfig, public hiveDesignPattern::CSingleton<CVisualizationConfig>
		{
		public:
			~CVisualizationConfig() override = default;

		private:
			CVisualizationConfig()
			{
				CVisualizationConfig::__defineAttributesV();

				const std::string ConfigPath = "Config/VisualizationConfig.xml";

				if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, this) != hiveConfig::EParseResult::SUCCEED)
				{
					_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
					return;
				}
			}

			void __defineAttributesV() override
			{
				_defineAttribute(SWITCH_UNWANTED_KEPT_MODE, hiveConfig::EConfigDataType::ATTRIBUTE_STRING);
				_defineAttribute(RECOVER_BACKGROUND_POINTS, hiveConfig::EConfigDataType::ATTRIBUTE_STRING);
				_defineAttribute(SWITCH_UNWANTED_DISCARD, hiveConfig::EConfigDataType::ATTRIBUTE_STRING);

				_defineAttribute(REMOVE_OUTLIER, hiveConfig::EConfigDataType::ATTRIBUTE_STRING);
				_defineAttribute(UNDO, hiveConfig::EConfigDataType::ATTRIBUTE_STRING);
				_defineAttribute(RADIUS_UP, hiveConfig::EConfigDataType::ATTRIBUTE_STRING);
				_defineAttribute(RADIUS_DOWN, hiveConfig::EConfigDataType::ATTRIBUTE_STRING);
				_defineAttribute(POINT_SIZE_DOWN, hiveConfig::EConfigDataType::ATTRIBUTE_STRING);
				_defineAttribute(POINT_SIZE_UP, hiveConfig::EConfigDataType::ATTRIBUTE_STRING);

				_defineAttribute(CIRCLE_MODE, hiveConfig::EConfigDataType::ATTRIBUTE_BOOL);
				_defineAttribute(AREA_MODE, hiveConfig::EConfigDataType::ATTRIBUTE_BOOL);
				_defineAttribute(AREA_PICK_CULLING, hiveConfig::EConfigDataType::ATTRIBUTE_BOOL);
				_defineAttribute(RUBBER_MODE, hiveConfig::EConfigDataType::ATTRIBUTE_BOOL);
				_defineAttribute(UNWANTED_MODE, hiveConfig::EConfigDataType::ATTRIBUTE_BOOL);
				_defineAttribute(REFRESH_IMMEDIATELY, hiveConfig::EConfigDataType::ATTRIBUTE_BOOL);
				
				_defineAttribute(SCREEN_CIRCLE_RADIUS, hiveConfig::EConfigDataType::ATTRIBUTE_DOUBLE);
				_defineAttribute(SCREEN_CIRCLE_HARDNESS, hiveConfig::EConfigDataType::ATTRIBUTE_DOUBLE);
				_defineAttribute(POINT_SHOW_SIZE, hiveConfig::EConfigDataType::ATTRIBUTE_DOUBLE);

				_defineAttribute(LITTER_HIGHLIGHT_COLOR, hiveConfig::EConfigDataType::ATTRIBUTE_VEC3I);
				_defineAttribute(BACKGROUND_HIGHLIGHT_COLOR, hiveConfig::EConfigDataType::ATTRIBUTE_VEC3I);

			}

			friend class hiveDesignPattern::CSingleton<CVisualizationConfig>;
		};
	}
}
