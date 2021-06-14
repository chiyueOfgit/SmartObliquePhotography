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

			virtual void runV(pcl::Indices& vioInputSet, EPointLabel vExpectLabel);

		private:
		};
	}
}