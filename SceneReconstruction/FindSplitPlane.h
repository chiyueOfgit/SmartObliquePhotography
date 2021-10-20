#pragma once

namespace hiveObliquePhotography::SceneReconstruction
{
	Eigen::Vector4f findSplitPlane(pcl::PointCloud<pcl::PointXYZ>::ConstPtr vLhs, pcl::PointCloud<pcl::PointXYZ>::ConstPtr vRhs);
}
