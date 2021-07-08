#pragma once

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		//TODO: 名字没想好
		class CScreenSpaceOperation
		{
		public:
			CScreenSpaceOperation(Eigen::Matrix4d vPvMatrix, const std::function<void(Eigen::Vector2f)>& vDistanceFunc)
				: m_PvMatrix(std::move(vPvMatrix)), m_DistanceFunc(vDistanceFunc)
			{}
			~CScreenSpaceOperation() = default;

			void cull(std::vector<pcl::index_t>& vioPointIndices, std::vector<float>& voPointDistance, const hiveConfig::CHiveConfig* vClusterConfig);
			void cullByDepth(std::vector<pcl::index_t>& vioPointIndices, const hiveConfig::CHiveConfig* vClusterConfig);
			void cullByRadius(std::vector<pcl::index_t>& vioPointIndices, std::vector<float>& voPointDistance, float vRadius, const hiveConfig::CHiveConfig* vClusterConfig);
			
		private:
			Eigen::Matrix4d m_PvMatrix;
			const std::function<void(Eigen::Vector2f)>& m_DistanceFunc;
		};
	}
}

