#pragma once
#include "PointClassifier.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class COutlierDetector : public IPointClassifier
		{
		public:
			COutlierDetector() = default;
			~COutlierDetector() = default;

			virtual void runV(pcl::Indices& vInputSet, EPointLabel vTargetLabel, const hiveConfig::CHiveConfig* vConfig);

		private:

		};
	}
}
