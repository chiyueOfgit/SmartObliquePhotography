#pragma once
#include "SpatialClusteringAlg.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class IPointCluster;

		class CMaxVisibilityClusterAlg : public CSpatialClusteringAlg
		{
		public:
			CMaxVisibilityClusterAlg() = default;
			~CMaxVisibilityClusterAlg() = default;

			void runV(const pcl::IndicesPtr& vioPointSet, EPointLabel vExpectLabel, const Eigen::Vector3f& vCameraPos, const Eigen::Matrix4d& vPvMatrix) ;
		};
	}
}
