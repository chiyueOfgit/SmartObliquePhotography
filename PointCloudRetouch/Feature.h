#pragma once

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class IFeature : public hiveDesignPattern::IProduct
		{
		public:
			IFeature() = default;
			virtual ~IFeature();

			virtual bool onProductCreatedV(const hiveConfig::CHiveConfig* vFeatureConfig)
			{
				_ASSERTE(vFeatureConfig); 
				m_pConfig = vFeatureConfig; 
				return true; 
			}

			virtual double generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter) = 0;
			virtual double evaluateFeatureMatchFactorV(pcl::index_t vInputPoint) = 0;

		protected:
			const hiveConfig::CHiveConfig* m_pConfig = nullptr;
		};
	}
}