#pragma once
#include "Image.h"

namespace hiveObliquePhotography
{
	class ITextureSaver :public hiveDesignPattern::IProduct
	{
	public:
		ITextureSaver() = default;
		~ITextureSaver() = default;

		virtual void saveDate2FileV(const CImage<Eigen::Vector3i> &vTextureData, const std::string &vFilePath) = 0;
	};
}