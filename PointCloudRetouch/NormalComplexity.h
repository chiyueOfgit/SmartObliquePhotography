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

		private:
			float m_AverageDon;
			pcl::search::KdTree<pcl::PointSurfel>::Ptr m_pTree = nullptr;
			float __calcPointCloudNormalComplexity(const std::vector<pcl::index_t>& vPointIndices);
		};
	}
}