#pragma once
#include "Feature.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
        struct LAB
        {
            float l;
            float a;
            float b;
        };

		struct SColorCluster
		{
			Eigen::Vector3i Centroid{ 0,0,0 };
			std::vector<pcl::index_t> Indices;
			float Coefficient = -FLT_MAX;
		};

		class CColorFeature : public IFeature
		{
		public:
			CColorFeature() = default;
			~CColorFeature() = default;

			void initV(const hiveConfig::CHiveConfig* vFeatureConfig) override;
			double generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter) override;
			double evaluateFeatureMatchFactorV(pcl::index_t vInputPoint) override;
			std::string outputDebugInfosV(pcl::index_t vIndex) const override;

			const auto& getMainBaseColors() const { return m_MainBaseColors; }

			const auto& getMainColorsNearestPoints() const { return m_NearestPoints; }

#ifdef _UNIT_TEST
			std::vector<Eigen::Vector3i> adaptiveKMeansCluster(const std::vector<Eigen::Vector3i>& vColorSet, std::size_t vK) { return __adaptiveColorClustering(vColorSet, vK); };
#endif
		private:
			std::vector<Eigen::Vector3i> __adaptiveColorClustering(const std::vector<Eigen::Vector3i>& vColorSet, std::size_t vMaxNumCluster) const;
			void __kMeansClustering(std::vector<SColorCluster>& voClusters, const std::vector<Eigen::Vector3i>& vColorSet, std::size_t vK, std::size_t vIterCount) const;
			void __fillClusterCoefficient(std::vector<SColorCluster>& vioClusters, const std::vector<Eigen::Vector3i>& vColorSet) const;

			float __calcColorDifferences(const Eigen::Vector3i& vLColor, const Eigen::Vector3i& vRColor) const;
			float __calculateCIEDE2000(const LAB& lab1, const LAB& lab2) const;
			LAB __RGB2LAB(const Eigen::Vector3f& vRGBColor) const;

			std::vector<Eigen::Vector3i> m_MainBaseColors;
			std::vector<pcl::index_t> m_NearestPoints;

			std::size_t m_NumCurrentScenePoints = 0;

			float m_ColorThreshold = 10.0f;
			std::size_t m_MaxNumMainColors = 5;
		};
	}

}
