#pragma once
#include "VisualizationExport.h"
#include "VisualizationConfig.h"

namespace hiveObliquePhotography
{
	namespace Visualization
	{
		VISUALIZATION_DECLSPEC void hiveInitVisualizer(PointCloud_t::Ptr vPointCloud, bool vIsInQt = true);

		VISUALIZATION_DECLSPEC void hiveResetVisualizer(PointCloud_t::Ptr vPointCloud = nullptr, bool vIsInQt = true);

		VISUALIZATION_DECLSPEC void hiveRefreshVisualizer(const std::vector<std::size_t>& vPointLabel, bool vResetCamera = false);

		VISUALIZATION_DECLSPEC void hiveRunVisualizerLoop();
		
		VISUALIZATION_DECLSPEC void hiveSetPointsColor(const std::vector<pcl::index_t>& vPointSet, const Eigen::Vector3i& vColor, bool vIsTemp = true);

		VISUALIZATION_DECLSPEC void hiveClearPointsColor();

		VISUALIZATION_DECLSPEC pcl::visualization::PCLVisualizer*& hiveGetPCLVisualizer();

		VISUALIZATION_DECLSPEC bool hiveGetVisualizationConfig(CVisualizationConfig*& voConfig);
	}
}