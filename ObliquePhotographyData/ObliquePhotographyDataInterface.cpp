#include "pch.h"
#include "ObliquePhotographyDataInterface.h"
#include "PointCloudScene.h"

using namespace hiveObliquePhotography;

//*****************************************************************
//FUNCTION: 
pcl::PointCloud<pcl::PointSurfel>::Ptr hiveObliquePhotography::hiveInitPointCloudScene(const std::vector<std::string>& vFileNameSet)
{
	_ASSERTE(!vFileNameSet.empty());

	return CPointCloudScene::getInstance()->loadScene(vFileNameSet);
}