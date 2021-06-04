#include "pch.h"
#include "VisualizerInterface.h"
#include "PointCloudVisualizer.h"

using namespace hiveObliquePhotography::visualizer;

VISUALIZER_DECLSPEC void hiveInitVisualizer(pcl::PointCloud<pcl::PointSurfel>::Ptr vPointCloud)
{
	CPointCloudVisualizer::getInstance()->init(vPointCloud);
}

VISUALIZER_DECLSPEC void hiveObliquePhotography::visualizer::hiveRefreshVisualizer(bool vResetCamera)
{
	CPointCloudVisualizer::getInstance()->refresh(vResetCamera);
}