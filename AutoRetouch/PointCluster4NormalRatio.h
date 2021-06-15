#pragma once
#include "PointCluster.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		using PointCloud_t = pcl::PointCloud<pcl::PointSurfel>;
		class CPointCluster4NormalRatio : public IPointCluster
		{
		public:
			CPointCluster4NormalRatio(const pcl::IndicesPtr& vPointIndices, EPointLabel vLabel);
			~CPointCluster4NormalRatio() override = default;

			double computeSimilarityV(pcl::index_t vPointIndex) const override;

		private:
			PointCloud_t::Ptr m_pPointCloud;
		};
	}
}
