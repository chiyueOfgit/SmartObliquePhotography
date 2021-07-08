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

			CPointCluster* createInitialCluster(const std::vector<pcl::index_t>& vUserMarkedRegion, float vHardness, const std::function<float(Eigen::Vector2f)>& vDistanceFunc, const Eigen::Matrix4d& vPvMatrix, EPointLabel vTargetLabel, const hiveConfig::CHiveConfig *vClusterConfig);

		private:
			void __divideUserSpecifiedRegion(const std::vector<pcl::index_t>& vUserMarkedRegion, const std::vector<float> vPointHardnessSet,  float vDivideThreshold, std::vector<pcl::index_t>& voFeatureGenerationSet, std::vector<pcl::index_t>& voValidationSet);
			void __generateHardness4EveryPoint(const std::vector<pcl::index_t>& vUserMarkedRegion, float vHardness, float vRadius, const Eigen::Vector2f& vCenter, const Eigen::Matrix4d& vPvMatrix, const std::pair<float, float>& vWindowSize, std::vector<float>& voPointHardnessSet, const hiveConfig::CHiveConfig* vClusterConfig);

			pcl::index_t __computeClusterCenter(const std::vector<pcl::index_t>& vUserMarkedRegion, const std::vector<float> vPointHardnessSet, const Eigen::Vector2f& vCenter, const Eigen::Matrix4d& vPvMatrix, const std::pair<float, float>& vWindowSize);

			const float m_DefaultPointSetDivideThreshold = 0.8;
		};
	}
}

