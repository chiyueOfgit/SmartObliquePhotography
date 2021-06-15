#pragma once
#include "PointCloudSaver.h"

namespace hiveObliquePhotography
{
	using Point_t = pcl::PointSurfel;

	class CPointCloudPCDSaver : public IPointCloudSaver
	{
		void saveDataToFile(const PointCloud_t& vPointCloud, const std::string& vFilePath) override;
	};
}