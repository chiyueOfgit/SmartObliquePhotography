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

#ifdef _UNIT_TEST
			const auto getVisitedTag() const { return m_pVisitedTag; }
#endif // _UNIT_TEST


		protected:
			PointCloud_t::Ptr m_pPointCloudScene = nullptr;

		private:
			const CPointLabelSet* m_pPointLabelSet = nullptr;
			bool* m_pVisitedTag = nullptr;

			virtual void __extraInitV(const hiveConfig::CHiveConfig* vConfig) {}
			virtual std::vector<pcl::index_t> __buildNeighborhoodV(pcl::index_t vSeed) = 0;
		};
	}
}