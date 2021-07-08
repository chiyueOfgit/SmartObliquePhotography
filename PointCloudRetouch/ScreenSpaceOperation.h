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
			CScreenSpaceOperation(const Eigen::Matrix4d& vPvMatrix, const std::pair<float, float>& vWindowSize)
				: m_PvMatrix(vPvMatrix), m_WindowSize(vWindowSize)
			{}
			~CScreenSpaceOperation() = default;

			void cullByDepth(std::vector<pcl::index_t>& vioPointIndices, const hiveConfig::CHiveConfig* vClusterConfig);
			void cullByRadius(std::vector<pcl::index_t>& vioPointIndices, float vRadius, const hiveConfig::CHiveConfig* vClusterConfig);
			
		private:
			Eigen::Matrix4d m_PvMatrix;
			std::pair<float, float> m_WindowSize;
		};
	}
}

