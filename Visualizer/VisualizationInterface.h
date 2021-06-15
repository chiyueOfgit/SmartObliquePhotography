#pragma once
#include "VisualizationExport.h"
#include "VisualizationConfig.h"

namespace hiveObliquePhotography
{
	namespace Visualization
	{
		using PointCloud_t = pcl::PointCloud<pcl::PointSurfel>;
		
		VISUALIZATION_DECLSPEC void hiveInitVisualizer(PointCloud_t::Ptr vPointCloud, bool vIsInQt = true);

		VISUALIZATION_DECLSPEC void hiveRefreshVisualizer(bool vResetCamera = false);

		VISUALIZATION_DECLSPEC void hiveRunVisualizerLoop();

		VISUALIZATION_DECLSPEC pcl::visualization::PCLVisualizer*& hiveGetPCLVisualizer();

		VISUALIZATION_DECLSPEC bool hiveGetVisualizationConfig(CVisualizationConfig*& voConfig);
	}
}