#pragma once
#include "PointCluster.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class CPointCluster4Score : public IPointCluster
		{
		public:
			CPointCluster4Score(const pcl::IndicesPtr& vPointIndices, EPointLabel vLabel);
			~CPointCluster4Score() override = default;

			double computeDistanceV(pcl::index_t vPointIndex) const override;

		private:
			Eigen::Vector3f m_Normal;
			Eigen::Vector3i m_Color;
			Eigen::Vector3f m_Position;
		};
	}
}
