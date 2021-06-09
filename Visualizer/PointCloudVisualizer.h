#pragma once
#include "pch.h"

namespace hiveObliquePhotography
{
	namespace Visualization
	{
		class CInteractionCallback;

		class CPointCloudVisualizer : public hiveDesignPattern::CSingleton<CPointCloudVisualizer>
		{
		public:
			~CPointCloudVisualizer();

			void init(pcl::PointCloud<pcl::PointSurfel>* vPointCloud);

			void reset(pcl::PointCloud<pcl::PointSurfel>* vPointCloud);

			void refresh(bool vResetCamera = false);

		private:
			CPointCloudVisualizer();

			pcl::visualization::PCLVisualizer* m_pPCLVisualizer = nullptr;
			CInteractionCallback* m_pCallback = nullptr;

			pcl::PointCloud<pcl::PointSurfel>* m_pSceneCloud = nullptr;

			friend class hiveDesignPattern::CSingleton<CPointCloudVisualizer>;
		};
	}
}
