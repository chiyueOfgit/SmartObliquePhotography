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

			void runV(const pcl::IndicesPtr& vioPointSet, EPointLabel vFinalLabel, const Eigen::Vector3f& vCameraPos, const std::vector<Eigen::Matrix4d>& vMatrices) ;
		};
	}
}
