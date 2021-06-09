#pragma once
#include "PointCluster.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class CPointCluster4VFH : public IPointCluster
		{
		public:
			CPointCluster4VFH(const std::vector<std::uint64_t>& vPointIndices, EPointLabel vLabel);
			virtual ~CPointCluster4VFH() = default;

			const std::set<std::uint64_t>& getClusterIndices() const { return m_PointIndices; }

			virtual double computeDistanceV(std::uint64_t vPointIndex) const override;

		private:
			void __computeVFHDescriptor(const std::vector<std::uint64_t>& vPointIndices, Eigen::Matrix<float, 308, 1>& voVFHDescriptor) const;

			std::set<std::uint64_t> m_PointIndices;
			Eigen::Matrix<float, 308, 1> m_VFHDescriptor;
		};
	}
}