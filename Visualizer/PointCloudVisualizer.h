#pragma once

namespace hiveObliquePhotography
{
	namespace visualizer
	{
		class CInteractionCallback;

		class CPointCloudVisualizer : public hiveDesignPattern::CSingleton<CPointCloudVisualizer>
		{
		public:
			~CPointCloudVisualizer();

			void init(pcl::PointCloud<pcl::PointSurfel>::Ptr vPointCloud);

			void reset(pcl::PointCloud<pcl::PointSurfel>::Ptr vPointCloud);

			void refresh(bool vResetCamera = false);

		private:
			CPointCloudVisualizer();

			pcl::visualization::PCLVisualizer* m_pPCLVisualizer;
			CInteractionCallback* m_pCallback;

			pcl::PointCloud<pcl::PointSurfel>::Ptr m_pSceneCloud;

			friend class hiveDesignPattern::CSingleton<CPointCloudVisualizer>;
		};
	}
}
