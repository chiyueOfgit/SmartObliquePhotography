#pragma once
#include "PointClassifier.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class ICompositeClassifier : public IPointClassifier
		{
		public:
			ICompositeClassifier() = default;
			~ICompositeClassifier() = default;

			void addClassifier(IPointClassifier* vClassifer);

		protected:
			void _ensembleResult();

		private:
			std::vector<IPointClassifier*> m_ClassifierSet;

			virtual EPointLabel __ensembleSingleResultV(const std::vector<SPointLabelChange>& vOverallResult) const = 0;
		};
	}
}
