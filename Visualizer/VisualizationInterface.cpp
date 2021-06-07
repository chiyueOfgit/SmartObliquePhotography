#include "pch.h"
#include "VisualizationInterface.h"
#include "PointCloudVisualizer.h"

using namespace hiveObliquePhotography::Visualization;

void hiveObliquePhotography::Visualization::hiveInitVisualizer(pcl::PointCloud<pcl::PointSurfel>* vPointCloud)
{
	CPointCloudVisualizer::getInstance()->init(vPointCloud);
}

void hiveObliquePhotography::Visualization::hiveRefreshVisualizer(bool vResetCamera)
{
	CPointCloudVisualizer::getInstance()->refresh(vResetCamera);
}