#include "pch.h"
#include "AutoRetouchInterface.h"

using namespace hiveObliquePhotography::AutoRetouch;

//*****************************************************************
//FUNCTION: 
void hiveObliquePhotography::AutoRetouch::hiveUndoLastOp()
{
	CPointCloudAutoRetouchScene::getInstance()->undoLastOp();
}

//*****************************************************************
//FUNCTION: 
void hiveObliquePhotography::AutoRetouch::hiveInitPointCloudScene(pcl::PointCloud<pcl::PointSurfel>* vPointCloud)
{
	CPointCloudAutoRetouchScene::getInstance()->init(vPointCloud);
}