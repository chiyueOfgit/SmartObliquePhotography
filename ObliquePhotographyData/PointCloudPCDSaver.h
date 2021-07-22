#pragma once
#include "PointCloudSaver.h"

namespace hiveObliquePhotography
{
	class CPointCloudPCDSaver : public IPointCloudSaver
	{
		void saveDataToFileV(const PointCloud_t& vPointCloud, const std::string& vFilePath) override;
	};
}