#pragma once

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class IFeature : public hiveDesignPattern::IProduct
		{
		public:
			IFeature() = default;
			virtual ~IFeature() = default;

			virtual void initV(const hiveConfig::CHiveConfig* vFeatureConfig)
			{
				_ASSERTE(vFeatureConfig);
				m_pConfig = vFeatureConfig;
			}

			virtual double generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter) = 0;
			virtual double evaluateFeatureMatchFactorV(pcl::index_t vInputPoint) = 0;
			virtual std::string outputDebugInfosV(pcl::index_t vIndex) const { return std::string(); }

		protected:
			const hiveConfig::CHiveConfig* m_pConfig = nullptr;
		};
	}
}