#pragma once
#include "PointCloudLoader.h"

namespace hiveObliquePhotography
{
	class CPointCloudPCDLoader : public IPointCloudLoader
	{
	private:
		int __loadDataFromFileV(const std::string& vFileName, PointCloud_t& voPointCloud) override;
	};
}
