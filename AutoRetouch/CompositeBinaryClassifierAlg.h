#pragma once
#include "CompositeClassifier.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class IPointCluster;
		class CBinaryClassifierAlg;

		class CCompositeBinaryClassifierAlg : protected CCompositeClassifier
		{
		public:
			CCompositeBinaryClassifierAlg() = default;
			~CCompositeBinaryClassifierAlg() override = default;

			void addBinaryClassifiers(const std::vector<std::string>& vClusterTypes);

			void init(CGlobalPointLabelSet* vGlobalLabelSet) { CCompositeClassifier::init(vGlobalLabelSet); }
			void run() { ensembleResult(); }

		private:
			virtual EPointLabel __ensembleSingleResultV(const std::vector<SPointLabelChange>& vOverallResult) const override;

			std::vector<std::string> m_ClusterTypes;
		};
	}
}
