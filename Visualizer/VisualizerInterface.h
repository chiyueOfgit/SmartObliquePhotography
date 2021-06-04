#pragma once
#include "VisualizerExport.h"

namespace hiveObliquePhotography
{
	namespace visualizer
	{
		VISUALIZER_DECLSPEC void hiveInitVisualizer(pcl::PointCloud<pcl::PointSurfel>::Ptr vPointCloud);

		VISUALIZER_DECLSPEC void hiveRefreshVisualizer(bool vResetCamera);
	}
}