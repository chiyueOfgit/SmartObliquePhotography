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

			bool onProductCreatedV(const hiveConfig::CHiveConfig* vFeatureConfig) override
			{
				_ASSERTE(vFeatureConfig);
				m_pConfig = vFeatureConfig;

				const auto& CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();
				pcl::PointCloud<pcl::PointXYZ>::Ptr pPointCloudScene(new pcl::PointCloud<pcl::PointXYZ>);
				for (size_t i = 0; i < CloudScene.getNumPoint(); i++)
				{
					const auto& Position = CloudScene.getPositionAt(i);
					pPointCloudScene->emplace_back(Position.x(), Position.y(), Position.z());
				}
				m_pTree.reset(new pcl::search::KdTree<pcl::PointXYZ>);
				m_pTree->setInputCloud(pPointCloudScene);
				
				return true;
		    }

		private:
			float m_AverageDon;
			pcl::search::KdTree<pcl::PointXYZ>::Ptr m_pTree = nullptr;
			float __calcPointCloudNormalComplexity(const std::vector<pcl::index_t>& vPointIndices);
		};
	}
}