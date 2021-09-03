#pragma once

namespace hiveObliquePhotograghy
{
	using Scalar_t = int;
	constexpr int Channel = 3;
	class ITextureLoader : public hiveDesignPattern::IProduct
	{
	public:
		ITextureLoader() = default;
		~ITextureLoader() override = default;
		
		virtual void loadTextureDataFromFile(Eigen::Matrix<Scalar_t, Channel, 1> &voTextureMatrix,const std::string& vFileName) = 0;
	};
}