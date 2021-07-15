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

			bool onProductCreatedV() override;
			
			double generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter) override;
			double evaluateFeatureMatchFactorV(pcl::index_t vInputPoint) override;
			virtual std::string outputDebugInfosV(pcl::index_t vIndex) const override;

#ifdef _UNIT_TEST
			double calcSinglePointNormalComplexity(pcl::index_t vInputPoint) const { return __calcSinglePointNormalComplexity(vInputPoint); }
#endif
		private:
			double m_AverageDon;
			pcl::search::Search<pcl::PointXYZ>::Ptr m_pTree;
			double __calcPointCloudNormalComplexity(const std::vector<pcl::index_t>& vPointIndices);
			double __calcSinglePointNormalComplexity(pcl::index_t vInputPoint) const;
		};
	}
}