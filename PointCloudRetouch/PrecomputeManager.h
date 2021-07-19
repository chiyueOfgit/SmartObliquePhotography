#pragma once
#include <boost/any.hpp>

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CPrecomputeManager
		{
		public:
			CPrecomputeManager() = default;
			~CPrecomputeManager() = default;

			void init(const hiveConfig::CHiveConfig* vConfig) { m_pClusterConfig = vConfig; }

			template<class T>
			void registerPrecompute(std::function<bool()> vPrecomputeFunc, const std::string& vPath, T* vContainer)
			{
				if (!hiveUtility::hiveLocateFile(vPath).empty())
				{
					T Temp;
					__deserialization<T>(vPath, &Temp);
					*vContainer = Temp;
				}
				else
				{
					auto pFunc = [=]()
					{
						if (vPrecomputeFunc())
							__serialization<T>(vPath, vContainer);
					};
					m_PrecomputeList.push_back(pFunc);
				}
			}

			const hiveConfig::CHiveConfig* getFeatureConfig(const std::string& vFeatureSig);

			void precompute();

			void clear();

		private:
			template<class T>
			void __serialization(const std::string& vPath, const T* vContainer) const
			{
				std::ofstream file(vPath.c_str());
				boost::archive::text_oarchive oa(file);
				oa& BOOST_SERIALIZATION_NVP(*vContainer);
				file.close();
			}

			template<class T>
			void __deserialization(const std::string& vPath, T* voContainer) const
			{
				std::ifstream file(vPath.c_str());
				boost::archive::text_iarchive ia(file);
				ia >> BOOST_SERIALIZATION_NVP(*voContainer);
				file.close();
			}
			
			std::vector<std::function<void()>> m_PrecomputeList;

			const hiveConfig::CHiveConfig* m_pClusterConfig;
		};
	}
}
