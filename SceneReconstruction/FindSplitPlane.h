#pragma once

namespace hiveObliquePhotography::SceneReconstruction
{
	Eigen::Vector4f findSplitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr vLhs, pcl::PointCloud<pcl::PointXYZ>::Ptr vRhs);
}