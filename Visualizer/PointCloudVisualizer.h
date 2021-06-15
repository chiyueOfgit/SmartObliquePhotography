#pragma once

namespace hiveObliquePhotography
{
	namespace Visualization
	{
		class CInteractionCallback;

		using PointCloud_t = pcl::PointCloud<pcl::PointSurfel>;
		class CPointCloudVisualizer : public hiveDesignPattern::CSingleton<CPointCloudVisualizer>
		{
		public:
			~CPointCloudVisualizer();

			void init(PointCloud_t::Ptr vPointCloud, bool vIsInQt = true);

			void reset(PointCloud_t::Ptr vPointCloud);

			void refresh(bool vResetCamera = false);

			void run();



		private:
			CPointCloudVisualizer();

			bool __parseConfigFile();

			pcl::visualization::PCLVisualizer* m_pPCLVisualizer = nullptr;
			CInteractionCallback* m_pCallback = nullptr;

			PointCloud_t::Ptr m_pSceneCloud = nullptr;

			friend class hiveDesignPattern::CSingleton<CPointCloudVisualizer>;
			friend class CInteractionCallback;
			friend void* hiveGetPCLVisualizer();
		};
	}
}
