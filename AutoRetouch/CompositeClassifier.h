#pragma once
#include "PointClassifier.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class CCompositeClassifier : protected IPointClassifier
		{
		public:
			CCompositeClassifier() = default;
			~CCompositeClassifier() = default;

			void ensembleResult();

			void init(CGlobalPointLabelSet* vGlobalLabelSet)
			{
				_ASSERTE(vGlobalLabelSet);
				m_pGlobalLabelSet = vGlobalLabelSet;
				m_pLocalLabelSet = m_pGlobalLabelSet->clone();
				_ASSERTE(m_pLocalLabelSet);
			}

			template<class TConcreteClassifier, class... TArgs>
			bool addClassifierAndExecute(TConcreteClassifier* vClassifier, TArgs&&... vArgs)
			{
				_ASSERTE(m_pLocalLabelSet && m_pGlobalLabelSet);
				_ASSERTE(vClassifer);

				try
				{
					if (m_pLocalLabelSet->getTimestamp() != m_pGlobalLabelSet->getTimestamp())
						m_pLocalLabelSet->update(m_pGlobalLabelSet);

					vClassifier->execute<TConcreteClassifier>(false, std::forward<TArgs>(vArgs)...);
					m_pLastClassifierResult = vClassifier->getResultIndices();
				}
				catch (std::runtime_error& e)
				{
					_HIVE_OUTPUT_WARNING(e.what());
				}
				catch (...)
				{
					_HIVE_OUTPUT_WARNING("Fail to execute to classifier due to unexpected error.");
				}

				m_ClassifierSet.emplace_back(vClassifier);
				return true;
			}

			template<class TConcreteClassifier, class... TArgs>
			bool addClassifierAndExecuteByLastIndices(TConcreteClassifier* vClassifier, TArgs&&... vArgs)
			{
				_ASSERTE(m_pLocalLabelSet && m_pGlobalLabelSet);
				_ASSERTE(vClassifer);

				try
				{
					if (m_pLocalLabelSet->getTimestamp() != m_pGlobalLabelSet->getTimestamp())
						m_pLocalLabelSet->update(m_pGlobalLabelSet);

					if (m_pLastClassifierResult)
						vClassifier->execute<TConcreteClassifier>(false, *m_pLastClassifierResult, std::forward<TArgs>(vArgs)...);
					else
						vClassifier->execute<TConcreteClassifier>(false, pcl::Indices{}, std::forward<TArgs>(vArgs)...);
					m_pLastClassifierResult = vClassifier->getResultIndices();
				}
				catch (std::runtime_error& e)
				{
					_HIVE_OUTPUT_WARNING(e.what());
			}
				catch (...)
				{
					_HIVE_OUTPUT_WARNING("Fail to execute to classifier [%1%] due to unexpected error.");
				}

				m_ClassifierSet.emplace_back(vClassifier);
				return true;
			}

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

		private:
			std::vector<IPointClassifier*> m_ClassifierSet;

			pcl::IndicesPtr m_pLastClassifierResult;

			virtual EPointLabel __ensembleSingleResultV(const std::vector<SPointLabelChange>& vOverallResult) const;
		};
	}
}
