#pragma once
#include "AutoRetouchCommon.h"
#include "PointLabel4Classfier.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class CPointLabel4Classfier;

		class IPointClassifier : public hiveDesignPattern::IProduct
		{
		public:
			IPointClassifier() = default;
			~IPointClassifier() { delete m_pLocalLabelSet; }

			virtual bool onProductCreatedV(CGlobalPointLabelSet* vGlobalLabelSet)
			{
				_ASSERTE(vGlobalLabelSet);
				m_pGlobalLabelSet = vGlobalLabelSet;
				m_pLocalLabelSet = m_pGlobalLabelSet->clone();
				_ASSERTE(m_pLocalLabelSet);
				return true;
			}

			template<class TConcreteClassifier, class... TArgs>
			bool execute(bool vApplyChange2GlobalLabelIntermediate, TArgs&&... vArgs)
			{
				return execute<TConcreteClassifier>(vApplyChange2GlobalLabelIntermediate, false, std::forward<TArgs>(vArgs)...);
			}
			
			template<class TConcreteClassifier, class... TArgs>
			bool execute(bool vApplyChange2GlobalLabelIntermediate, bool vClusterFlag, TArgs&&... vArgs)
			{
				_ASSERTE(m_pLocalLabelSet && m_pGlobalLabelSet);
				
				try
				{
					if (m_pLocalLabelSet->getTimestamp() != m_pGlobalLabelSet->getTimestamp())
						m_pLocalLabelSet->update(m_pGlobalLabelSet);

					m_pLocalLabelSet->startRecord();

					TConcreteClassifier* pClassifer = dynamic_cast<TConcreteClassifier*>(this);
					if (!pClassifer) _THROW_RUNTIME_ERROR(_FORMAT_STR1("Fail to execute the classifier [%1%] due to the failure of casting it to the concrete classifier object.", m_Name));

					pClassifer->runV(std::forward<TArgs>(vArgs)...);

					m_pLocalLabelSet->stopRecord();
					m_pLocalLabelSet->dumpPointLabelChangeRecord(m_PointLabelChangeRecord);

					if (vApplyChange2GlobalLabelIntermediate)
					{
						m_pGlobalLabelSet->applyPointLabelChange(m_PointLabelChangeRecord, vClusterFlag);
					}

					return true;
				}
				catch (std::runtime_error& e)
				{
					_HIVE_OUTPUT_WARNING(e.what());
				}
				catch (...)
				{
					_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to execute to classifier [%1%] due to unexpected error.", m_Name));
				}
				m_pLocalLabelSet->stopRecord();
				m_pLocalLabelSet->reset(m_PointLabelChangeRecord);
				return false;
			}

			const std::vector<SPointLabelChange>& getResult() const { return m_PointLabelChangeRecord; }
			pcl::IndicesPtr getResultIndices() const
			{
				pcl::IndicesPtr ResultIndices(new pcl::Indices);
				ResultIndices->reserve(m_PointLabelChangeRecord.size());
				for (auto& LabelChange : m_PointLabelChangeRecord)
					ResultIndices->push_back(LabelChange.Index);
				return ResultIndices;
			}

#ifdef _UNIT_TEST
			auto& getGlobalLabelSet() const { return m_pGlobalLabelSet; }
#endif
		
		protected:
			CGlobalPointLabelSet* m_pGlobalLabelSet = nullptr;
			CLocalPointLabelSet*  m_pLocalLabelSet = nullptr;

		private:
			std::vector<SPointLabelChange> m_PointLabelChangeRecord;
			std::string m_Name;
		};
	}
}
