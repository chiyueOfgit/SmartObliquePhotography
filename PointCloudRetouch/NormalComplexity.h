#pragma once
#include "Feature.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CNormalComplexity : public IFeature
		{
		public:
			CNormalComplexity() = default;
			~CNormalComplexity() override = default;

			//bool onProductCreatedV(const hiveConfig::CHiveConfig* vFeatureConfig) override;
			
			double generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter) override;
			double evaluateFeatureMatchFactorV(pcl::index_t vInputPoint) override;
			virtual std::string outputDebugInfosV(pcl::index_t vIndex) const override;

#ifdef _UNIT_TEST
			float calcSinglePointNormalComplexity(pcl::index_t vInputPoint) { return __calcSinglePointNormalComplexity(vInputPoint); }
#endif
		private:
			float m_AverageDon;
			pcl::PointCloud<pcl::PointNormal> m_DonCloud;
			float __calcPointCloudNormalComplexity(const std::vector<pcl::index_t>& vPointIndices);
			float __calcSinglePointNormalComplexity(pcl::index_t vInputPoint);
		};
	}
}