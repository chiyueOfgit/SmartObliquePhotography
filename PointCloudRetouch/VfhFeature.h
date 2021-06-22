#pragma once
#include "Feature.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		const std::size_t VfhDimension = 308;

		class CVfhFeature : public IFeature
		{
		public:
			CVfhFeature();
			~CVfhFeature() = default;

			virtual double generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter) override;
			virtual double evaluateFeatureMatchFactorV(pcl::index_t vInputPoint) override;

		private:
			void __computeVfhDescriptor(const std::vector<pcl::index_t>& vPointIndices, Eigen::Matrix<float, VfhDimension, 1>& voVfhDescriptor) const;
			double __KernelDotVfhDescriptor(const Eigen::Matrix<float, VfhDimension, 1>& vLVfh, const Eigen::Matrix<float, VfhDimension, 1>& vRVfh, std::size_t vBlockSize = 1) const;

			PointCloud_t::Ptr m_pCloud = nullptr;
			pcl::search::KdTree<pcl::PointSurfel>::Ptr m_pTree = nullptr;
			Eigen::Matrix<float, VfhDimension, 1> m_DeterminantVfhDescriptor;
			double m_BaseDotResult = 0.0;
			std::size_t m_KernelSize = 5;
		};
	}

}
