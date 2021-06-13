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

#ifdef _UNIT_TEST
			std::vector<std::vector<SPointLabelChange>> getResults() const
			{
				std::vector<std::vector<SPointLabelChange>> Results;
				for (auto pClassifier : m_ClassifierSet)
				{
					Results.push_back(pClassifier->getResult());
				}
				return Results;
			}
#endif // _UNIT_TEST

		protected:
			void _ensembleResult();

		private:
			std::vector<IPointClassifier*> m_ClassifierSet;

			virtual EPointLabel __ensembleSingleResultV(const std::vector<SPointLabelChange>& vOverallResult) const = 0;
		};
	}
}
