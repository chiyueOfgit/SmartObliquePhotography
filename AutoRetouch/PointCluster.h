#pragma once

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class IPointCluster
		{
		public:
			IPointCluster() = default;
			IPointCluster(EPointLabel vLabel) : m_Label(vLabel) {}
			virtual ~IPointCluster() = default;

			EPointLabel getClusterLabel() const { return m_Label; }

			void setClusterAABB(const SBox& vAABB) { m_AABB = vAABB; }
			const SBox& getClusterAABB() const { return m_AABB; }

			const pcl::IndicesPtr& getClusterIndices() const { return m_PointIndices; }
			
			virtual double computeSimilarityV(pcl::index_t vPointIndex) const = 0;

		private:
			SBox m_AABB;

			EPointLabel m_Label = EPointLabel::UNDETERMINED;
		};
	}
}
