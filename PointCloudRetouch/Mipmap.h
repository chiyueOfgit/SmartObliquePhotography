#pragma once

using namespace hiveObliquePhotography::PointCloudRetouch;

namespace Utility {

	void generateResultImage(const Eigen::Matrix<Eigen::Vector3i, -1, -1>& vTexture, const std::string& vOutputImagePath);

	Eigen::Matrix<Eigen::Vector3i, -1, -1> getMipMap(const Eigen::Matrix<Eigen::Vector3i, -1, -1>& vTexture);
}
