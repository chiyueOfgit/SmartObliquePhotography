#pragma once

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class IFeature;

		class CPointCluster
		{
		public:
			CPointCluster() = default;
			~CPointCluster() = default;

			bool init(const hiveConfig::CHiveConfig* vConfig, std::uint32_t vClusterCenter, EPointLabel vLabel, const std::vector<pcl::index_t>& vFeatureGenerationSet, const std::vector<pcl::index_t>& vValidationSet, std::uint32_t vCreationTimestamp);
			bool isBelongingTo(double vProbability) const;

			double evaluateProbability(pcl::index_t vInputPoint) const;

			const std::vector<pcl::index_t>& getCoreRegion() const { return m_ClusterCoreRegion; }

			std::uint32_t getClusterIndex() const { return m_CreationTimestamp;} //使用唯一的时间戳作为clustser index

			EPointLabel getLabel() const { return m_Label; }

			void outputDebugInfos(pcl::index_t vIndex) const;

		protected:
			std::vector<IFeature*> m_FeatureSet;
			std::vector<double> m_FeatureWeightSet;

		private:
			std::uint32_t m_CreationTimestamp;
			const hiveConfig::CHiveConfig* m_pConfig = nullptr;
			EPointLabel  m_Label;
			pcl::index_t m_ClusterCenter;
			std::vector<pcl::index_t> m_ClusterCoreRegion;
			float m_ExpectProbability;

			void __createFeatureObjectSet();
		};
	}
}