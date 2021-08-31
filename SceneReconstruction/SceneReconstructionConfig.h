#pragma once
#include <common/HiveConfig.h>
#include <common/Singleton.h>
#include <common/ConfigInterface.h>

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		const std::string OCTREE_DEPTH = "OCTREE_DEPTH";
		
		class CSceneReconstructionConfig : public hiveConfig::CHiveConfig, public hiveDesignPattern::CSingleton<CSceneReconstructionConfig>
		{
		public:
			~CSceneReconstructionConfig() override = default;

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
				_defineAttribute(OCTREE_DEPTH, hiveConfig::EConfigDataType::ATTRIBUTE_INT);
			}

			friend class hiveDesignPattern::CSingleton<CSceneReconstructionConfig>;
		};
	}
}
