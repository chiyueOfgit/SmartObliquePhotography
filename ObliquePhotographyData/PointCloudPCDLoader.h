#pragma once
#include "PointCloudLoader.h"

namespace hiveObliquePhotography
{
	class CPointCloudPCDLoader : public IPointCloudLoader
	{
	private:
		bool __loadDataFromFileV(const std::string& vFileName, pcl::PointCloud<pcl::PointSurfel>& voPointCloud) override;
	};
}
