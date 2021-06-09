#pragma once
#include "VisualizationExport.h"

namespace hiveObliquePhotography
{
	namespace Visualization
	{
		VISUALIZATION_DECLSPEC void hiveInitVisualizer(pcl::PointCloud<pcl::PointSurfel>::Ptr vPointCloud);

		VISUALIZATION_DECLSPEC void hiveRefreshVisualizer(bool vResetCamera = false);

		VISUALIZATION_DECLSPEC void hiveRunVisualizerLoop();

		VISUALIZATION_DECLSPEC void* hiveGetPCLVisualizer();
	}
}