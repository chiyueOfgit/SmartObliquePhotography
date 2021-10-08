#pragma once
#include "PointCloudLoader.h"

namespace hiveObliquePhotography
{
	class CPointCloudPLYLoader : public IPointCloudLoader
	{
	private:
		int __loadDataFromFileV(const std::string& vFileName, PointCloud_t::Ptr voPointCloud) override;
	};
}
