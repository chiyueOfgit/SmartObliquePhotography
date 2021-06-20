#pragma once
#include "PointCluster.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class CPointCluster4NormalRatio : public IPointCluster
		{
		public:
			CPointCluster4NormalRatio(const pcl::IndicesPtr& vPointIndices, EPointLabel vLabel);
			~CPointCluster4NormalRatio() override = default;

			double computeSimilarityV(pcl::index_t vPointIndex) const override;

		private:
			pcl::IndicesPtr m_PointIndices;
		};
	}
}
