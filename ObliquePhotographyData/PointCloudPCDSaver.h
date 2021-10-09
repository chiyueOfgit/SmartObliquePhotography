#pragma once
#include "PointCloudSaver.h"

namespace hiveObliquePhotography
{
	class CPointCloudPCDSaver : public IPointCloudSaver
	{
		void saveDataToFileV(PointCloud_t::Ptr vPointCloud, const std::string& vFilePath) override;
	};
}