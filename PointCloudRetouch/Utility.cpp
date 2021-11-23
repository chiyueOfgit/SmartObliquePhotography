#include "pch.h"
#include "Utility.h"

#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

bool hiveObliquePhotography::PointCloudRetouch::saveTexture(const std::string& vPath, const hiveObliquePhotography::CImage<float>& vTexture, bool vIsReverse)
{
	const auto Width = vTexture.getWidth();
	const auto Height = vTexture.getHeight();
	const auto BytesPerPixel = 1;
	auto ResultImage = new unsigned char[Width * Height * BytesPerPixel];
	for (auto i = 0; i < Height; i++)
		for (auto k = 0; k < Width; k++)
		{
			auto I = i;
			if (vIsReverse)
				I = Height - 1 - I;
			auto Offset = (I * Width + k) * BytesPerPixel;
			ResultImage[Offset] = static_cast<int>(vTexture.getColor(i, k));
		}

	stbi_write_png(vPath.c_str(), Width, Height, BytesPerPixel, ResultImage, 0);
	stbi_image_free(ResultImage);
}