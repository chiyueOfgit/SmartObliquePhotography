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

		class CColorFeature : public IFeature
		{
		public:
			CColorFeature() = default;
			~CColorFeature() = default;

			virtual double generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter) override;
			virtual double evaluateFeatureMatchFactorV(pcl::index_t vInputPoint) override;
			virtual std::string outputDebugInfosV(pcl::index_t vIndex) const override;

			const auto& getMainBaseColors() const { return m_MainBaseColors; }

			const auto& getMainColorsNearestPoints() const { return m_NearestPoints; }

#ifdef _UNIT_TEST
			std::vector<Eigen::Vector3i> adjustKMeansCluster(const std::vector<Eigen::Vector3i>& vColorSet, std::size_t vK) { return __adjustKMeansCluster(vColorSet, vK); };
#endif
		private:
			std::vector<Eigen::Vector3i> __adjustKMeansCluster(const std::vector<Eigen::Vector3i>& vColorSet, std::size_t vK) const;

			float __calcColorDifferences(const Eigen::Vector3i& vLColor, const Eigen::Vector3i& vRColor) const;
			float __calculateCIEDE2000(const LAB& lab1, const LAB& lab2) const;
			LAB __RGB2LAB(const Eigen::Vector3f& vRGBColor) const;

			std::vector<Eigen::Vector3i> m_MainBaseColors;
			std::vector<pcl::index_t> m_NearestPoints;

			pcl::search::KdTree<pcl::PointXYZ>::Ptr m_pTree = nullptr;
			std::size_t m_NumCurrentScenePoints = 0;

			float m_ColorThreshold = 10.0f;
			std::size_t m_MaxNumMainColors = 5;
			float m_MinReduceRatio = 0.8f;
		};
	}

}
