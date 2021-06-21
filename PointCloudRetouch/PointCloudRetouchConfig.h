#pragma once

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CPointCloudRetouchConfig : public hiveConfig::CHiveConfig
		{
		public:
			CPointCloudRetouchConfig() = default;
			~CPointCloudRetouchConfig() = default;

		private:
			virtual void __defineAttributesV() override;
		};
	}
}

