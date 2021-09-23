#pragma once
#include "PointCloudRetouchExport.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class RETOUCH_DECLSPEC CPointCloudRetouchConfig : public hiveConfig::CHiveConfig
		{
		public:
			CPointCloudRetouchConfig() = default;
			~CPointCloudRetouchConfig() = default;

		private:
			virtual void __defineAttributesV() override;
		};
	}
}

