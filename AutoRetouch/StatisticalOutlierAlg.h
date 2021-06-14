#pragma once
#include "OutlierDetectingAlg.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class IPointCluster;

		class CStaOutlierDetectingAlg : public COutlierDetectingAlg
		{
		public:
			CStaOutlierDetectingAlg() = default;
			~CStaOutlierDetectingAlg() = default;

			void runV(pcl::Indices& vioInputSet, EPointLabel vExpectLabel) override;
		};
	}
}
