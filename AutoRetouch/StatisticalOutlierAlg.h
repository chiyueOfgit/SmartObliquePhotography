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

			virtual void runV(std::vector<std::uint64_t>& vioInputSet, EPointLabel vFinalLabel);
		};
	}
}
