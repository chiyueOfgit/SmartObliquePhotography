#pragma once
#include "PointClassifier.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CGroundHeightFilter : public IPointClassifier
		{
		public:
			CGroundHeightFilter() = default;
			~CGroundHeightFilter() = default;

			virtual void runV(pcl::Indices& vInputSet, EPointLabel vExpectLabel, const hiveConfig::CHiveConfig* vConfig);

		private:

		};
	}
}

