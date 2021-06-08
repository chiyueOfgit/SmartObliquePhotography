#pragma once
#include "SpatialClusteringAlg.h"
#include <pcl/visualization/pcl_visualizer.h>

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

			virtual void  runV(std::vector<std::uint64_t>& vioInputSet, EPointLabel vFinalLabel, const pcl::visualization::Camera& vCamera) ;
		};
	}
}
