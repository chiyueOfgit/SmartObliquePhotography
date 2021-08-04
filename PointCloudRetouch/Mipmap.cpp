#include "pch.h"
#include "Mipmap.h"
#include <cmath>
#include <numbers>

#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

float __getGaussianWeight(float vRadius, float vSigma)
{
	// Considering that normalization is required later, the coefficient is not calculated
	return std::pow(std::numbers::e, -vRadius * vRadius / (2 * vSigma * vSigma));
}

Eigen::Matrix<float, -1, -1> __getGaussianKernal(int vKernalSize, float vSigma)
{
	if (!vSigma)
		vSigma = 0.3 * ((vKernalSize - 1) * 0.5 - 1) + 0.8;

	float SumWeight = 0.0;
	Eigen::Matrix<float, -1, -1> GaussianKernal(vKernalSize, vKernalSize);

	for (int i = 0; i < vKernalSize; i++)
		for (int k = 0; k < vKernalSize; k++)
		{
			GaussianKernal(i, k) = __getGaussianWeight(std::abs((vKernalSize - 1) / 2 - i) + std::abs((vKernalSize - 1) / 2 - k), vSigma);
			SumWeight += GaussianKernal.coeff(i, k);
		}

	GaussianKernal /= SumWeight;

	return GaussianKernal;
}

float __executeGaussianFilter(const Eigen::Matrix<Eigen::Vector3i, -1, -1>& vTexture, const Eigen::Matrix<float, -1, -1>& vGaussianKernal, int vChannel, int vRow, int vCol)
{
	float Value = 0.0;
	int GaussianKernalRowIndex = -1;
	int GaussianKernalColIndex = -1;
	for (int i = vRow - (vGaussianKernal.rows() - 1) / 2; i <= vRow + (vGaussianKernal.rows() - 1) / 2; i++)
	{
		GaussianKernalRowIndex++;
		for (int k = vCol - (vGaussianKernal.cols() - 1) / 2; k <= vCol + (vGaussianKernal.cols() - 1) / 2; k++)
		{
			GaussianKernalColIndex++;
			if (i < 0 || k < 0 || i >= vTexture.rows() || k >= vTexture.cols())
				continue;
			Value += vGaussianKernal.coeff(GaussianKernalRowIndex, GaussianKernalColIndex) * vTexture.coeff(i, k)[vChannel];
		}
		GaussianKernalColIndex = -1;
	}

	return Value;
}

void __executeGaussianBlur(const Eigen::Matrix<Eigen::Vector3i, -1, -1>& vTexture, Eigen::Matrix<Eigen::Vector3i, -1, -1>& voMipmap)
{
	auto GaussianKernal = __getGaussianKernal(3, 0);
	for (int i = 0; i < voMipmap.rows(); i++)
		for (int k = 0; k < voMipmap.cols(); k++)
			for (int Channel = 0; Channel < 3; Channel++)
				voMipmap(i, k)[Channel] = __executeGaussianFilter(vTexture, GaussianKernal, Channel, i * 2, k * 2);
}

void Utility::generateResultImage(const Eigen::Matrix<Eigen::Vector3i, -1, -1>& vTexture, const std::string& vOutputImagePath)
{
	const auto Width = vTexture.cols();
	const auto Height = vTexture.rows();
	const auto BytesPerPixel = 3;
	auto ResultImage = new unsigned char[Width * Height * BytesPerPixel];
	for (auto i = 0; i < Height; i++)
		for (auto k = 0; k < Width; k++)
		{
			auto Offset = (i * Width + k) * BytesPerPixel;
			ResultImage[Offset] = vTexture.coeff(i, k)[0];
			ResultImage[Offset + 1] = vTexture.coeff(i, k)[1];
			ResultImage[Offset + 2] = vTexture.coeff(i, k)[2];
		}

	stbi_write_png(vOutputImagePath.c_str(), Width, Height, BytesPerPixel, ResultImage, 0);
	stbi_image_free(ResultImage);
}

Eigen::Matrix<Eigen::Vector3i, -1, -1> Utility::getMipMap(const Eigen::Matrix<Eigen::Vector3i, -1, -1>& vTexture)
{
	Eigen::Matrix<Eigen::Vector3i, -1, -1> Mipmap((vTexture.rows() + 1) / 2, (vTexture.cols() + 1) / 2);
	__executeGaussianBlur(vTexture, Mipmap);
	return Mipmap;
}
