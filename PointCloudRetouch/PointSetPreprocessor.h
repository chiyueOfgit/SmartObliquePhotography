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
			
			void cullByDepth(std::vector<pcl::index_t>& vioPointSet, const Eigen::Matrix4d& vPvMatrix, const Eigen::Vector3f& vViewPos);
			void cullBySdf(std::vector<pcl::index_t>& vioPointSet, const Eigen::Matrix4d& vPvMatrix, const std::function<float(Eigen::Vector2f)>& vSignedDistanceFunc);
		private:
			std::pair<Eigen::Vector2f, Eigen::Vector2f> __computeBoundingBoxOnNdf(const std::vector<pcl::index_t>& vPointSet, const Eigen::Matrix4d& vPvMatrix);


		};
	}
}
