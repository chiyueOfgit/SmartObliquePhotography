#pragma once
#include <common/HiveConfig.h>
#include <common/Singleton.h>

namespace hiveObliquePhotography
{
	namespace QTInterface
	{
		const std::string DOCKWIDGETTITLEBAR_BACKGROUNDCOLOR = "DOCKWIDGETTITLEBAR_BACKGROUNDCOLOR";
		const std::string DOCKWIDGETTITLEBAR_FONTCOLOR = "DOCKWIDGETTITLEBAR_FONTCOLOR";
		const std::string DOCKWIDGETTITLEBAR_FONTSIZE = "DOCKWIDGETTITLEBAR_FONTSIZE";


		class CQInterfaceConfig : public hiveConfig::CHiveConfig, public hiveDesignPattern::CSingleton<CQInterfaceConfig>
		{
		public:
			~CQInterfaceConfig() override = default;

		private:
			CQInterfaceConfig() { CQInterfaceConfig::__defineAttributesV(); }

			void __defineAttributesV() override
			{
				_defineAttribute(DOCKWIDGETTITLEBAR_BACKGROUNDCOLOR, hiveConfig::EConfigDataType::ATTRIBUTE_VEC4I);
				_defineAttribute(DOCKWIDGETTITLEBAR_FONTCOLOR, hiveConfig::EConfigDataType::ATTRIBUTE_VEC4I);
				_defineAttribute(DOCKWIDGETTITLEBAR_FONTSIZE, hiveConfig::EConfigDataType::ATTRIBUTE_INT);


			}

			friend class hiveDesignPattern::CSingleton<CQInterfaceConfig>;
		};
	}
}
