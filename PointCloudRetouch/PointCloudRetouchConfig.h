#pragma once
#include "PointCloudRetouchExport.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		RETOUCH_DECLSPEC class CPointCloudRetouchConfig : public hiveConfig::CHiveConfig
		{
		public:
			CPointCloudRetouchConfig() = default;
			~CPointCloudRetouchConfig() = default;

		private:
			RETOUCH_DECLSPEC virtual void __defineAttributesV() override;
		};
	}
}

