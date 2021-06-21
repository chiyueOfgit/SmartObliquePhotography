#pragma once
#include "Feature.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CVfhFeature : public IFeature
		{
		public:
			virtual double generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter) override;
			virtual double evaluateFeatureMatchFactorV(pcl::index_t vInputPoint) override;

		private:
			void __computeVfhDescriptor(const std::vector<pcl::index_t>& vPointIndices, Eigen::Matrix<float, 308, 1>& voVfhDescriptor) const;

			Eigen::Matrix<float, 308, 1> m_VFHDescriptor;
		};
	}

}
