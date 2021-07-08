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
			CScreenSpaceOperation(Eigen::Matrix4d vPvMatrix, Eigen::Vector3f vViewPos, Eigen::Vector2i vWindowSize, Eigen::Vector2i vLeftUp, Eigen::Vector2i vRightDown)
				: m_PvMatrix(std::move(vPvMatrix)), m_ViewPos(std::move(vViewPos)), m_WindowSize(std::move(vWindowSize)), m_LeftUp(std::move(vLeftUp)), m_RightDown(std::move(vRightDown))
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

