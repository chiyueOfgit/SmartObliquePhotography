#pragma once

namespace hiveObliquePhotography
{
	namespace Visualization
	{
		class CInteractionCallback;

		class CPointCloudVisualizer : public hiveDesignPattern::CSingleton<CPointCloudVisualizer>
		{
		public:
			~CPointCloudVisualizer();

			void init(pcl::PointCloud<pcl::PointSurfel>::Ptr vPointCloud);

			void reset(pcl::PointCloud<pcl::PointSurfel>::Ptr vPointCloud);

			void refresh(bool vResetCamera = false);

			void run();

		private:
			CPointCloudVisualizer();

			pcl::visualization::PCLVisualizer* m_pPCLVisualizer = nullptr;
			CInteractionCallback* m_pCallback = nullptr;

			pcl::PointCloud<pcl::PointSurfel>::Ptr m_pSceneCloud = nullptr;

			friend class hiveDesignPattern::CSingleton<CPointCloudVisualizer>;
			friend class CInteractionCallback;
		};
	}
}
