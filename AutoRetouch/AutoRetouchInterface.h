#pragma once
#include "AutoRetouchExport.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		AUTORETOUCH_DECLSPEC void hiveInitPointCloudScene(pcl::PointCloud<pcl::PointSurfel>* vPointCloud);

		AUTORETOUCH_DECLSPEC bool hiveExecuteRegionGrow();
	}
}