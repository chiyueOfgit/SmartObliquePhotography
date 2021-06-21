#pragma once

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class IPointClassifier : public hiveDesignPattern::IProduct
		{
		public:
			IPointClassifier() = default;
			~IPointClassifier() = default;

			template<class TConcreteClassifier, class... TArgs>
			bool execute(TArgs&&... vArgs)
			{
				m_IsClassifyDone = false;

				try
				{
					TConcreteClassifier* pClassifer = dynamic_cast<TConcreteClassifier*>(this);
					if (!pClassifer) _THROW_RUNTIME_ERROR(_FORMAT_STR1("Fail to execute the classifier [%1%] due to the failure of casting it to the concrete classifier object.", getProductSig()));

					pClassifer->runV(std::forward<TArgs>(vArgs)...);

					m_IsClassifyDone = true;
					return true;
				}
				catch (std::runtime_error& e)
				{
					_HIVE_OUTPUT_WARNING(_FORMAT_STR2("Fail to execute to classifier [%1%] due to error [%2%].", getProductSig(), e.what()));
				}
				catch (...)
				{
					_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to execute to classifier [%1%] due to unexpected error.", getProductSig()));
				}
				return false;
			}

		private:
			bool m_IsClassifyDone = false;
		};
	}
}