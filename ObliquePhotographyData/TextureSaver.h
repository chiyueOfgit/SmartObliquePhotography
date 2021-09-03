#pragma once

namespace hiveObliquePhotography
{
	using Scalar_t = int;
	constexpr int Channel = 3;
	class ITextureSaver :public hiveDesignPattern::IProduct
	{

		ITextureSaver() = default;
		~ITextureSaver() override = default;

		virtual void saveDate2FileV(const Eigen::Matrix<Scalar_t,Channel,1> &vTextureData, const std::string &vFilePath) = 0;
	};
}