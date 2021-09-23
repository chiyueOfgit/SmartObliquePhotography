#pragma once

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CPointCluster;

		class CInitialClusterCreator
		{
		public:
			CInitialClusterCreator() = default;
			~CInitialClusterCreator() = default;

			CPointCluster* createInitialCluster(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix, const std::function<double(Eigen::Vector2d)>& vHardnessFunc, EPointLabel vTargetLabel, const hiveConfig::CHiveConfig *vClusterConfig);

		private:
			pcl::index_t __computeCenterIndex(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix) const;
			std::vector<double> __generateHardness4EveryPoint(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix, const std::function<double(Eigen::Vector2d)>& vHardnessFunc) const;
			void __divideUserSpecifiedRegion(const std::vector<pcl::index_t>& vUserMarkedRegion, const std::vector<double>& vPointHardnessSet, float vDivideThreshold, std::vector<pcl::index_t>& voFeatureGenerationSet, std::vector<pcl::index_t>& voValidationSet);

			const float m_DefaultPointSetDivideThreshold = 0.8f;
		};
	}
}

