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

			CPointCluster* createInitialCluster(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix, float vHardness, EPointLabel vTargetLabel, const hiveConfig::CHiveConfig *vClusterConfig);

		private:
			std::vector<float> __computeDistanceSetFromCenter(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix);
			std::vector<float> __generateHardness4EveryPoint(const std::vector<float>& vDistanceSetFromCenter, float vHardness);
			void __divideUserSpecifiedRegion(const std::vector<pcl::index_t>& vUserMarkedRegion, const std::vector<float>& vPointHardnessSet, float vDivideThreshold, std::vector<pcl::index_t>& voFeatureGenerationSet, std::vector<pcl::index_t>& voValidationSet);

			const float m_DefaultPointSetDivideThreshold = 0.8f;
		};
	}
}

