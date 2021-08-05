#pragma once

using namespace hiveObliquePhotography::PointCloudRetouch;

namespace Utility {

	Eigen::Matrix<Eigen::Vector3i, -1, -1> getMipMap(const Eigen::Matrix<Eigen::Vector3i, -1, -1> &vTexture);
}
