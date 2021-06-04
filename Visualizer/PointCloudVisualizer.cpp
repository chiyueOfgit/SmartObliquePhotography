#include "pch.h"
#include "PointCloudVisualizer.h"
#include "pcl/io/pcd_io.h"
//#include "PointCloudAutoRetouchScene.h"

using namespace hiveObliquePhotography::visualizer;

CPointCloudVisualizer::CPointCloudVisualizer()
{
	setBackgroundColor(0.2, 0.2, 0.2);
}

CPointCloudVisualizer::~CPointCloudVisualizer()
{

}

//*****************************************************************
//FUNCTION: 
void CPointCloudVisualizer::refresh(bool vResetCamera)
{
	pcl::PointCloud<pcl::PointSurfel>::Ptr pCloud(new pcl::PointCloud<pcl::PointSurfel>);

	pcl::io::loadPCDFile("Panda.pcd", *pCloud);

	addPointCloud<pcl::PointSurfel>(pCloud);

	if (vResetCamera)
		resetCamera();
	else
		updateCamera();
}