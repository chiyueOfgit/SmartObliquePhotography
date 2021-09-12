#pragma once

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CPointSetPreprocessor
		{
		public:
			CPointSetPreprocessor() = default;
			~CPointSetPreprocessor() = default;
			
			void cullByDepth(std::vector<pcl::index_t>& vioPointSet, const Eigen::Matrix4d& vPvMatrix, const Eigen::Vector3d& vViewPos);
			void cullBySdf(std::vector<pcl::index_t>& vioPointSet, const Eigen::Matrix4d& vPvMatrix, const std::function<double(Eigen::Vector2d)>& vSignedDistanceFunc);
		private:
			std::pair<Eigen::Vector2d, Eigen::Vector2d> __computeBoundingBoxOnNdc(const std::vector<pcl::index_t>& vPointSet, const Eigen::Matrix4d& vPvMatrix);

		};
	}
}
