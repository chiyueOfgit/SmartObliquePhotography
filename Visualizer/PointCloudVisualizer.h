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

			void init(PointCloud_t::Ptr vPointCloud, bool vIsInQt = true);

			void reset(PointCloud_t::Ptr vPointCloud);

			void refresh(bool vResetCamera = false);

			void run();

		private:
			CPointCloudVisualizer();

			pcl::visualization::PCLVisualizer* m_pPCLVisualizer = nullptr;
			CInteractionCallback* m_pCallback = nullptr;

			PointCloud_t::Ptr m_pSceneCloud = nullptr;

			friend class hiveDesignPattern::CSingleton<CPointCloudVisualizer>;
			friend class CInteractionCallback;
			friend pcl::visualization::PCLVisualizer*& hiveGetPCLVisualizer();
		};
	}
}
