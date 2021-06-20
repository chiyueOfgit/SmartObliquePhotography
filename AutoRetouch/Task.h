#pragma once

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class IPointClassifier;
		class CGlobalPointLabelSet;

		class ITask
		{
		public:
			ITask() = default;
			virtual ~ITask();

			[[nodiscard]] bool init(const hiveConfig::CHiveConfig* vTaskConfig, CGlobalPointLabelSet *vGlobapPointLabelSet);

			template<class... TArgs>
			bool execute()
			{
				_ASSERTE(m_pGlobalPointLabelSet);

				m_pGlobalPointLabelSet->snapshot();

				try
				{
					__runV(std::forward<TArgs>(vArgs)...);
					m_pGlobalPointLabelSet->recordChange();
					return true;
				}
				catch (std::runtime_error* e)
				{
					_HIVE_OUTPUT_WARNING(_FORMAT_STR2("Fail to execute task [%1%] due to the error [%2%].", m_Name, e->what()));

				}
				catch (...)
				{
					_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to execute task [%1%] due to unexpected error", m_Name));
				}
				m_pGlobalPointLabelSet->restoreSnapshot();
				return false;
			}

		private:
			std::string m_Name;
			const hiveConfig::CHiveConfig* m_pTaskConfig = nullptr;
			CGlobalPointLabelSet* m_pGlobalPointLabelSet = nullptr;
			std::vector<IPointClassifier*> m_ClassifierSet;
		};
	}
}

