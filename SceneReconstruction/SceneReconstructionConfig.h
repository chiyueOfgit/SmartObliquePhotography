#pragma once
#include "SceneReconstructionCommon.h"
#include <common/HiveConfig.h>
#include <common/Singleton.h>
#include <common/ConfigInterface.h>

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		class CSceneReconstructionConfig : public hiveConfig::CHiveConfig, public hiveDesignPattern::CSingleton<CSceneReconstructionConfig>
		{
		public:
			~CSceneReconstructionConfig() override = default;

			const hiveConfig::CHiveConfig* getSubConfigByName(const std::string& vName) { return findSubconfigByName(vName); }

		private:
			CSceneReconstructionConfig()
			{
				CSceneReconstructionConfig::__defineAttributesV();

				const std::string ConfigPath = "Config/SceneReconstructionConfig.xml";
				if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, this) != hiveConfig::EParseResult::SUCCEED)
				{
					_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
					return;
				}
			}

			void __defineAttributesV() override
			{
				_defineAttribute(KEYWORD::SURFACE, hiveConfig::EConfigDataType::ATTRIBUTE_SUBCONFIG);
				_defineAttribute(KEYWORD::OCTREE_DEPTH, hiveConfig::EConfigDataType::ATTRIBUTE_INT);

				_defineAttribute(KEYWORD::TEXCOORD, hiveConfig::EConfigDataType::ATTRIBUTE_SUBCONFIG);

				_defineAttribute(KEYWORD::TEXTURE, hiveConfig::EConfigDataType::ATTRIBUTE_SUBCONFIG);
				_defineAttribute(KEYWORD::RESOLUTION, hiveConfig::EConfigDataType::ATTRIBUTE_VEC2I);
				_defineAttribute(KEYWORD::SURFEL_RADIUS, hiveConfig::EConfigDataType::ATTRIBUTE_DOUBLE);
				_defineAttribute(KEYWORD::NUM_SAMPLE, hiveConfig::EConfigDataType::ATTRIBUTE_INT);
				_defineAttribute(KEYWORD::DISTANCE_THRESHOLD, hiveConfig::EConfigDataType::ATTRIBUTE_DOUBLE);
				_defineAttribute(KEYWORD::SEARCH_RADIUS, hiveConfig::EConfigDataType::ATTRIBUTE_DOUBLE);
				_defineAttribute(KEYWORD::WEIGHT_COEFFICIENT, hiveConfig::EConfigDataType::ATTRIBUTE_DOUBLE);

				_defineAttribute(KEYWORD::SUTURE, hiveConfig::EConfigDataType::ATTRIBUTE_SUBCONFIG);
			}

			friend class hiveDesignPattern::CSingleton<CSceneReconstructionConfig>;
		};
	}
}