#pragma once

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		struct SPointLabel
		{
			EPointLabel PointLabel = EPointLabel::UNDETERMINED;
			std::uint32_t ClusterIndex = 0;
			double Probability = 0;
		};

		class CPointLabelSet
		{
		public:
			CPointLabelSet() = default;
			~CPointLabelSet() = default;

			void tagPointLabel(pcl::index_t vPoint, EPointLabel vTargetLabel, std::uint32_t vClusterIndex, double vClusterBelongingProbability);
			void tagCoreRegion4Cluster(const std::vector<pcl::index_t>& vTargetPointSet, EPointLabel vTargetLabel, std::uint32_t vClusterIndex);
			void init(std::size_t vSize);

#ifdef _UNIT_TEST
			const std::vector<SPointLabel>& getLabelSet() const { return m_LabelSet; }
#endif

			EPointLabel   getLabelAt(std::size_t vIndex) const { _ASSERTE(vIndex < m_LabelSet.size()); return m_LabelSet[vIndex].PointLabel; }
			std::uint32_t getClusterIndexAt(std::size_t vIndex) const { _ASSERTE(vIndex < m_LabelSet.size()); return m_LabelSet[vIndex].ClusterIndex; }

			double getClusterBelongingProbabilityAt(std::size_t vIndex) const { _ASSERTE(vIndex < m_LabelSet.size()); return m_LabelSet[vIndex].Probability; }

		private:
			std::vector<SPointLabel> m_LabelSet;
		};
	}
}