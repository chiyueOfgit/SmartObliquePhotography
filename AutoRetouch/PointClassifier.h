#pragma once
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

			virtual bool onProductCreatedV(CGlobalPointLabelSet* vGlobalLabelSet, const hiveConfig::CHiveConfig *vConfig, bool vOperateOnLocalLabelSet)
			{
				_ASSERTE(vGlobalLabelSet && vConfig);
				m_pGlobalLabelSet = vGlobalLabelSet;
				m_pConfig = vConfig;
				m_IsOperatingOnLocalLabelSet = vOperateOnLocalLabelSet;
				return true;
			}

			template<class TConcreteClassifier, class... TArgs>
			bool execute(TArgs&&... vArgs)
			{
				_ASSERTE(m_pGlobalLabelSet);
				
				m_IsClassifyDone = false;

				try
				{
					if (m_IsOperatingOnLocalLabelSet)
					{
						m_pLocalLabelSet = m_pGlobalLabelSet->clone();
						_ASSERTE(m_pLocalLabelSet);
					}

					TConcreteClassifier* pClassifer = dynamic_cast<TConcreteClassifier*>(this);
					if (!pClassifer) _THROW_RUNTIME_ERROR(_FORMAT_STR1("Fail to execute the classifier [%1%] due to the failure of casting it to the concrete classifier object.", m_Name));

					pClassifer->runV(std::forward<TArgs>(vArgs)...);

					__onClassifyDoneV();

					m_IsClassifyDone = true;
					return true;
				}
				catch (std::runtime_error& e)
				{
					_HIVE_OUTPUT_WARNING(_FORMAT_STR2("Fail to execute to classifier [%1%] due to error [%2%].", m_Name, e.what()));
				}
				catch (...)
				{
					_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to execute to classifier [%1%] due to unexpected error.", m_Name));
				}
				return false;
			}

			const CLocalPointLabelSet* getLocalPointLabelSet() { _ASSERTE(m_IsOperatingOnLocalLabelSet); return m_pLocalLabelSet; }

		protected:
			CGlobalPointLabelSet* m_pGlobalLabelSet = nullptr;
			CLocalPointLabelSet*  m_pLocalLabelSet = nullptr;
			const hiveConfig::CHiveConfig* m_pConfig = nullptr;

		private:
			bool m_IsClassifyDone = false;
			bool m_IsOperatingOnLocalLabelSet = false;
			std::vector<SPointLabelChange> m_PointLabelChangeRecord;
			std::string m_Name;

			virtual void __onClassifyDoneV() {}
		};
	}
}
