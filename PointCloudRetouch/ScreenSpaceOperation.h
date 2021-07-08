#pragma once

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CPointCluster;

		//TODO: 名字没想好
		class CScreenSpaceOperation
		{
		public:
			CScreenSpaceOperation(const Eigen::Matrix4d& vPvMatrix, const Eigen::Vector3f& vViewPos, const Eigen::Vector2i& vWindowSize, const Eigen::Vector2i& vLeftUp, const Eigen::Vector2i& vRightDown)
				: m_PvMatrix(vPvMatrix), m_ViewPos(vViewPos), m_WindowSize(vWindowSize), m_LeftUp(vLeftUp), m_RightDown(vRightDown)
			{}
			~CScreenSpaceOperation() = default;

			void cullByDepth(std::vector<pcl::index_t>& vioPointIndices, const hiveConfig::CHiveConfig* vClusterConfig);
			void cullByRadius(std::vector<pcl::index_t>& vioPointIndices, std::vector<float>& voPointDistance, float vRadius, const hiveConfig::CHiveConfig* vClusterConfig);
			
		private:
			Eigen::Matrix4d m_PvMatrix;
			Eigen::Vector3f m_ViewPos;
			Eigen::Vector2i m_WindowSize;
			Eigen::Vector2i m_LeftUp;
			Eigen::Vector2i m_RightDown;
		};
	}
}

