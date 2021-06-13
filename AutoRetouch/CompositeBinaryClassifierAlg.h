#pragma once
#include "CompositeClassifier.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class IPointCluster;

		class CCompositeBinaryClassifierAlg : public CCompositeClassifier
		{
		public:
			CCompositeBinaryClassifierAlg() = default;
			~CCompositeBinaryClassifierAlg() override = default;

			bool init();

		private:
			virtual EPointLabel __ensembleSingleResultV(const std::vector<SPointLabelChange>& vOverallResult) const override;
		};
	}
}
