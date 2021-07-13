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
			bool onProductCreatedV(const hiveConfig::CHiveConfig* vFeatureConfig)
			{
				_ASSERTE(vFeatureConfig);
				m_pConfig = vFeatureConfig;

				return true;	
		    }
			
#ifdef _UNIT_TEST
			double calcSinglePointNormalComplexity(pcl::index_t vIndex) { return __calcSinglePointNormalComplexity(vIndex); }
#endif		
		private:
			float m_AverageDon;
			pcl::search::KdTree<pcl::PointSurfel>::Ptr m_pTree = nullptr;
			
			float __calcSinglePointNormalComplexity(pcl::index_t vIndex);
			float __calcPointCloudNormalComplexity(pcl::PointCloud<pcl::PointXYZ>::Ptr vCloud);
		};
	}
}