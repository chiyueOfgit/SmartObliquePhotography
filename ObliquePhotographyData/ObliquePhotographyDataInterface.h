#pragma once
#include "ObliquePhotographyDataExport.h"
#include <vector>
#include <string>

namespace hiveObliquePhotography
{
	OPDATA_DECLSPEC bool hiveInitPointCloudScene(const std::vector<std::string>& vFileNameSet, const std::string& vDataLoaderSig);

}