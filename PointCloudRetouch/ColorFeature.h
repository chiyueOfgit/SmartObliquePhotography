#pragma once
#include "Feature.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CColorFeature : public IFeature
		{
		public:
			~CColorFeature() = default;

			virtual double generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter) override;
			virtual double evaluateFeatureMatchFactorV(pcl::index_t vInputPoint) override;

		private:
			void __computeMainColors(const std::vector<pcl::index_t>& vPointIndices, std::vector<Eigen::Vector3i>& vMainColors, std::size_t vK);
			std::vector<Eigen::Vector3i> __kMeansCluster(const std::vector<Eigen::Vector3i>& vData, std::size_t vK) const;

			PointCloud_t::Ptr m_pCloud = nullptr;
			std::vector<Eigen::Vector3i> m_MainColors;
		};
	}

}
