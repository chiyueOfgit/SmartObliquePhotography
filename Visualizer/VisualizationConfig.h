#pragma once
#include <common/HiveConfig.h>
#include <common/Singleton.h>

namespace hiveObliquePhotography
{
	namespace Visualization
	{
		const std::string VIEW_BINARY_RESULT = "VIEW_BINARY_RESULT";
		const std::string SWITCH_UNWANTED_DISCARD = "SWITCH_UNWANTED_DISCARD";
		const std::string SWITCH_BINARY_GROWING = "SWITCH_BINARY_GROWING";
		const std::string SWITCH_BINARY_CLUSTER_LABEL = "SWITCH_BINARY_CLUSTER_LABEL";
		const std::string POINT_SHOW_SIZE = "POINT_SHOW_SIZE";

		class CVisualizationConfig : public hiveConfig::CHiveConfig, public hiveDesignPattern::CSingleton<CVisualizationConfig>
		{
		public:
			~CVisualizationConfig() override = default;

		private:
			CVisualizationConfig() { CVisualizationConfig::__defineAttributesV(); }

			void __defineAttributesV() override
			{
				_defineAttribute(VIEW_BINARY_RESULT, hiveConfig::EConfigDataType::ATTRIBUTE_STRING);
				_defineAttribute(SWITCH_UNWANTED_DISCARD, hiveConfig::EConfigDataType::ATTRIBUTE_STRING);
				_defineAttribute(SWITCH_BINARY_GROWING, hiveConfig::EConfigDataType::ATTRIBUTE_STRING);
				_defineAttribute(SWITCH_BINARY_CLUSTER_LABEL, hiveConfig::EConfigDataType::ATTRIBUTE_STRING);
				_defineAttribute(POINT_SHOW_SIZE, hiveConfig::EConfigDataType::ATTRIBUTE_INT);

			}

			friend class hiveDesignPattern::CSingleton<CVisualizationConfig>;
		};
	}
}
