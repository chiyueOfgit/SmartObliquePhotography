#pragma once
#include "PointCluster.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class CPointCluster4VFH : public IPointCluster
		{
		public:
			CPointCluster4VFH(const pcl::Indices& vPointIndices, EPointLabel vLabel);
			virtual ~CPointCluster4VFH() = default;

			const std::set<pcl::index_t>& getClusterIndices() const { return m_PointIndices; }

			virtual double computeDistanceV(pcl::index_t vPointIndex) const override;

		private:
			void __computeVFHDescriptor(const pcl::Indices& vPointIndices, Eigen::Matrix<float, 308, 1>& voVFHDescriptor) const;

			std::set<pcl::index_t> m_PointIndices;
			Eigen::Matrix<float, 308, 1> m_VFHDescriptor;
		};
	}
}