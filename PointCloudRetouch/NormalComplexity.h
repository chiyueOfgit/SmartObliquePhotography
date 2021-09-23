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

			void initV(const hiveConfig::CHiveConfig* vFeatureConfig) override;
			double generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter) override;
			double evaluateFeatureMatchFactorV(pcl::index_t vInputPoint) override;
			std::string outputDebugInfosV(pcl::index_t vIndex) const override;

			bool precomputeSceneCloudNormalComplexity();

			auto* getPtr2Container() { return &m_NormalComplexity; }

#ifdef _UNIT_TEST
			double calcSinglePointNormalComplexity(pcl::index_t vInputPoint) const { return __calcSinglePointNormalComplexity(vInputPoint); }
#endif
		private:
			double m_Radius;
			double m_AverageDon;
			std::vector<double> m_NormalComplexity;
			pcl::search::Search<pcl::PointXYZ>::Ptr m_pTree;
			double __calcPointCloudNormalComplexity(const std::vector<pcl::index_t>& vPointIndices);
			double __calcSinglePointNormalComplexity(pcl::index_t vInputPoint) const;
			void __buildSearchTree();

		};
	}
}