#pragma once
#include "PointClassifier.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class COutlierDetectingAlg : public IPointClassifier
		{
		public:
			COutlierDetectingAlg() = default;
			~COutlierDetectingAlg() = default;

			virtual void runV(const std::vector<std::uint64_t>& vioInputSet, EPointLabel vFinalLabel);

		private:
		};
	}
}