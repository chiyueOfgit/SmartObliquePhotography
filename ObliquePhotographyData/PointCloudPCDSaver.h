#pragma once
#include "PointCloudSaver.h"

namespace hiveObliquePhotography
{
	class CPointCloudPCDSaver : public IPointCloudSaver
	{
		void saveDataToFile(const PointCloud_t& vPointCloud, const std::string& vFilePath) override;
	};
}