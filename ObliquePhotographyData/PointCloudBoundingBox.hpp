#pragma once
#include <pcl/features/moment_of_inertia_estimation.h>

namespace hiveObliquePhotography
{
	template <typename PointType>
		requires pcl::traits::has_xyz_v<PointType>
	std::pair<PointType, PointType> getAABB(typename pcl::PointCloud<PointType>::ConstPtr vCloud)
	{
		PointType MinPoint, MaxPoint;
		pcl::MomentOfInertiaEstimation<PointType> FeatureExtractor;
		FeatureExtractor.setInputCloud(vCloud);
		FeatureExtractor.compute();
		FeatureExtractor.getAABB(MinPoint, MaxPoint);
		return { MinPoint , MaxPoint };
	}
}
