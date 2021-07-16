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
			std::vector<std::function<bool()>> m_PrecomputeList;
		};
	}
}
