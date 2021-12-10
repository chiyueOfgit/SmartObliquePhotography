#pragma once
#include "common/Product.h"
#include "SmartOPCommon.h"

namespace hiveObliquePhotography
{
	class IPointCloudFileLoader : public hiveDesignPattern::IProduct
	{
	public:
		IPointCloudFileLoader();
		~IPointCloudFileLoader();

		SUnorganizedPointCloud* load(const std::string& vFileName);

	private:
		virtual SUnorganizedPointCloud* __loadV(const std::string& vFileName) = 0;
	};
}

