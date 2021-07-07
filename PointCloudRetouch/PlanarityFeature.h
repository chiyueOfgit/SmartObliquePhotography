#pragma once
#include "Feature.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CPlanarityFeature : public IFeature
		{
		public:
			CPlanarityFeature() = default;
			~CPlanarityFeature() override = default;

			double generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter) override;
			double evaluateFeatureMatchFactorV(pcl::index_t vInputPoint) override;
			virtual std::string outputDebugInfosV(pcl::index_t vIndex) const override;
#ifdef _UNIT_TEST
			Eigen::Vector4f fitPlane(PointCloud_t::Ptr vCloud) { return __fitPlane(vCloud); };
#endif
		private:
			//(normalized.normal.x, normalized.normal.y, normalized.normal.z, distance)
			Eigen::Vector4f m_Plane;
			std::pair<float, float> m_Peak;
			float m_Tolerance = 0.12f;

			PointCloud_t::Ptr __createPositionCloud(const std::vector<pcl::index_t>& vIndexSet);
			Eigen::Vector4f __fitPlane(PointCloud_t::Ptr vCloud) const;
			std::pair<float, float> __computePeakDistance(PointCloud_t::Ptr vCloud, Eigen::Vector4f vPlane);
		};
	}
}
