#pragma once
#include "Feature.h"
#include "PointCloudRetouchExport.h"

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
			std::string outputDebugInfosV(pcl::index_t vIndex) const override;

			//TODO：有的可以提到common
			RETOUCH_DECLSPEC static Eigen::Vector4f fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr vCloud, double vDistanceThreshold, const Eigen::Vector3f& vUp);
			RETOUCH_DECLSPEC static float smoothAttenuation(float vFrom, float vTo, float vX);
		
		private:
			//(normalized.normal.x, normalized.normal.y, normalized.normal.z, distance)
			Eigen::Vector4f m_Plane;
			const float m_DistanceThreshold = 1.0f;
		};
	}
}
