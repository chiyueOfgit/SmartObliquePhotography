#pragma once

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CNeighborhood;
		class CPointLabelSet;

		class INeighborhoodBuilder : public hiveDesignPattern::IProduct
		{
		public:
			INeighborhoodBuilder() = default;
			virtual ~INeighborhoodBuilder();

			virtual bool onProductCreatedV(const hiveConfig::CHiveConfig* vConfig, PointCloud_t::Ptr vPointCloudScene, const CPointLabelSet* vPointLabelSet);

			std::vector<pcl::index_t> buildNeighborhood(pcl::index_t vSeed, std::uint32_t vSeedClusterIndex);
			void reset();

		protected:
			PointCloud_t::Ptr m_pPointCloudScene = nullptr;

		private:
			const CPointLabelSet* m_pPointLabelSet = nullptr;
			std::vector<std::vector<std::uint32_t>> m_ClusterTag;

			virtual void __extraInitV(const hiveConfig::CHiveConfig* vConfig) {}
			virtual std::vector<pcl::index_t> __buildNeighborhoodV(pcl::index_t vSeed) = 0;
		};
	}
}