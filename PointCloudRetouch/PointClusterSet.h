#pragma once

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CPointCluster;

		class CPointClusterSet
		{
		public:
			CPointClusterSet() = default;
			~CPointClusterSet();

			void reset(std::uint64_t vTimestamp);

			std::size_t getNumCluster() const { return m_ClusterSet.size(); }

		private:
			std::vector<CPointCluster*> m_ClusterSet;
		};
	}
}