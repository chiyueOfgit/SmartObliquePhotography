#pragma once

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CPrecomputeManager
		{
		public:
			CPrecomputeManager() = default;
			~CPrecomputeManager() = default;

			void registerPrecomputeFunction(std::function<bool()> vPrecomputeFunc);

			void runAllPrecompute();

			void clear();

		private:
			template<class T>
			void __serialization(const std::string& vPath, const T& vContainer) const
			{
				std::ofstream file(vPath.c_str());
				boost::archive::text_oarchive oa(file);
				oa& BOOST_SERIALIZATION_NVP(vIndices);
				file.close();
			}

			template<class T>
			void __deserialization(const std::string& vPath, T& voContainer) const
			{
				std::ifstream file(vPath.c_str());
				boost::archive::text_iarchive ia(file);
				ia >> BOOST_SERIALIZATION_NVP(voContainer);
				file.close();
			}
			
			std::vector<std::function<bool()>> m_PrecomputeList;
			
		};
	}
}
