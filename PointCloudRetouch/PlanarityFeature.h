#pragma once
#include "Feature.h"
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>

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
			static Eigen::Vector4f fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr vCloud, double vDistanceThreshold, const Eigen::Vector3f& vUp)
			{
				Eigen::VectorXf Coeff;
				pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr ModelPlane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(vCloud));
				pcl::RandomSampleConsensus<pcl::PointXYZ> Ransac(ModelPlane);
				Ransac.setDistanceThreshold(vDistanceThreshold);
				Ransac.computeModel();
				Ransac.getModelCoefficients(Coeff);
				if (!Coeff.size())
					return { 0, 0, 0, 0 };
				const Eigen::Vector3f Normal(Coeff.x(), Coeff.y(), Coeff.z());
				if (Normal.dot(vUp) < 0.0f)
					Coeff *= -1.0f;
				return Coeff / Normal.norm();
			}
			static std::pair<float, float> computePeakDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr vCloud, const Eigen::Vector4f& vPlane)
			{
				float MinDistance = FLT_MAX;
				float MaxDistance = -FLT_MAX;
				for (auto& i : *vCloud)
				{
					MinDistance = std::min(MinDistance, vPlane.dot(i.getVector4fMap()));
					MaxDistance = std::max(MaxDistance, vPlane.dot(i.getVector4fMap()));
				}

				return { MinDistance, MaxDistance };
			}
			static float smoothAttenuation(float vFrom, float vTo, float vX)
			{
				auto Factor = (vX - vFrom) / (vTo - vFrom);

				if (Factor >= 1 || Factor <= 0)
					return 0;

				//x^4 - 2 * x^2 + 1 
				Factor *= Factor;
				return Factor * (Factor - 2) + 1;
			}
		
		private:
			//(normalized.normal.x, normalized.normal.y, normalized.normal.z, distance)
			Eigen::Vector4f m_Plane;
			//min, max
			std::pair<float, float> m_Peak;
		};
	}
}
