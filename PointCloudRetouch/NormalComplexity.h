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
			
			double generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter) override;
			double evaluateFeatureMatchFactorV(pcl::index_t vInputPoint) override;
#ifdef _UNIT_TEST
			double calcSinglePointNormalComplexity(pcl::index_t vIndex, PointCloud_t::Ptr vCloud) { return __calcSinglePointNormalComplexity(vIndex, vCloud); }
#endif		
		private:
			double __calcSinglePointNormalComplexity(pcl::index_t vIndex, PointCloud_t::Ptr vCloud);
		};
	}
}