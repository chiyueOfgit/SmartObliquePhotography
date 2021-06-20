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

			bool init(const hiveConfig::CHiveConfig* vConfig, std::uint32_t vClusterCenter, const std::vector<pcl::index_t>& vFeatureGenerationSet, const std::vector<pcl::index_t>& vValidationSet, std::uint32_t vCreationTimestamp);

			double evaluateProbability(pcl::index_t vInputPoint, std::uint32_t vCurrentTimestamp);

			const std::vector<pcl::index_t>& getCoreRegion() const { return m_ClusterCoreRegion; }

		protected:
			std::vector<IFeature*> m_FeatureSet;
			std::vector<double> m_FeatureWeightSet;

		private:
			std::uint32_t m_CreationTimestamp;
			const hiveConfig::CHiveConfig* m_pConfig = nullptr;
			pcl::index_t m_ClusterCenter;
			std::vector<pcl::index_t> m_ClusterCoreRegion;

			void __createFeatureObjectSet();
		};
	}
}