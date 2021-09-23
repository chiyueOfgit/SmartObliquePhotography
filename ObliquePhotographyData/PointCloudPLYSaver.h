#pragma once
#include "PointCloudSaver.h"

namespace hiveObliquePhotography
{
	class CPointCloudPLYSaver : public IPointCloudSaver
	{
		void saveDataToFileV(const PointCloud_t& vPointCloud, const std::string& vFilePath) override;
	};
}