#pragma once
#include "PointClassifier.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class CRegionGrowingAlg : public IPointClassifier
		{
		public:
			CRegionGrowingAlg() = default;
			~CRegionGrowingAlg() = default;

			virtual void runV(const std::vector<std::uint64_t>& vSeedSet, EPointLabel vSeedLabel);

		private:
		};
	}
}
