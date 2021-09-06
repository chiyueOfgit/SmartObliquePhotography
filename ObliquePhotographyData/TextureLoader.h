#pragma once
#include "Image.h"

namespace hiveObliquePhotography
{
	class ITextureLoader : public hiveDesignPattern::IProduct 
	{
	public:
		ITextureLoader() = default;
		~ITextureLoader() = default;
		
		virtual void loadTextureDataFromFileV(CImage<Eigen::Vector3i>& voTextureData, const std::string& vFileName) = 0;
	};
}