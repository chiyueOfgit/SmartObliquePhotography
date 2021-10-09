#pragma once
#include "PointCloudSaver.h"

namespace hiveObliquePhotography
{
	class CPointCloudPLYSaver : public IPointCloudSaver
	{
		void saveDataToFileV(PointCloud_t::Ptr vPointCloud, const std::string& vFilePath) override;
	};
}