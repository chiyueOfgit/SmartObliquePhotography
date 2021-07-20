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

			void addCluster(CPointCluster* vCluster) { m_ClusterSet.push_back(vCluster); }

			const CPointCluster* getLastCluster() const { return !m_ClusterSet.empty() ? m_ClusterSet.back() : nullptr; }
			void reset();

			std::size_t getNumCluster() const { return m_ClusterSet.size(); }

		private:
			std::vector<CPointCluster*> m_ClusterSet;
		};
	}
}