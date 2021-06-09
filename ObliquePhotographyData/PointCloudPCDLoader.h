#pragma once
#include "PointCloudLoader.h"

namespace hiveObliquePhotography
{
	using PointCloud_t = pcl::PointCloud<pcl::PointSurfel>;
	
	class CPointCloudPCDLoader : public IPointCloudLoader
	{
	private:
		bool __loadDataFromFileV(const std::string& vFileName, PointCloud_t& voPointCloud) override;
	};
}
