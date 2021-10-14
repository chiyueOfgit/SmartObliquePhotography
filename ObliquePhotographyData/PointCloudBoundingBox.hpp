#pragma once
#include <pcl/features/moment_of_inertia_estimation.h>

namespace hiveObliquePhotography
{
	template <typename PointType>
		requires pcl::traits::has_xyz_v<PointType>
	auto getAabb(std::shared_ptr<const pcl::PointCloud<PointType>> vCloud) -> std::pair<PointType, PointType>
	{
		PointType MinPoint, MaxPoint;
		pcl::MomentOfInertiaEstimation<PointType> FeatureExtractor;
		FeatureExtractor.setInputCloud(vCloud);
		FeatureExtractor.compute();
		FeatureExtractor.getAABB(MinPoint, MaxPoint);
		return { MinPoint , MaxPoint };
	}
}
