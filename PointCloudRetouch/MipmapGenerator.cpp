#include "pch.h"
#include "MipmapGenerator.h"
#include <cmath>
#include <numbers>

#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

template <typename Scalar_t, unsigned Channel>
using Texture_t = Eigen::Matrix<Eigen::Matrix<Scalar_t, Channel, 1>, Eigen::Dynamic, Eigen::Dynamic>;

template <typename Scalar_t, unsigned Channel>
Texture_t<Scalar_t, Channel> CMipmapGenerator<Scalar_t, Channel>::getMipmap(const Texture_t& vTexture)
{
	Texture_t Mipmap((vTexture.rows() + 1) / 2, (vTexture.cols() + 1) / 2);
	__executeGaussianBlur(vTexture, Mipmap);
	return Mipmap;
}

template <typename Scalar_t, unsigned Channel>
void CMipmapGenerator<Scalar_t, Channel>::__executeGaussianBlur(const Texture_t &vTexture, Texture_t &voMipmap)
{
	auto GaussianKernal = __getGaussianKernal(3, 0);
	for (int i = 0; i < voMipmap.rows(); i++)
		for (int k = 0; k < voMipmap.cols(); k++)
			for (int m = 0; m < Channel; m++)
				voMipmap(i, k)[m] = __executeGaussianFilter(vTexture, GaussianKernal, m, i * 2, k * 2);
}

template <typename Scalar_t, unsigned Channel>
float CMipmapGenerator<Scalar_t, Channel>::__executeGaussianFilter(const Texture_t &vTexture, const Eigen::Matrix<float, -1, -1> &vGaussianKernal, int vChannel, int vRow, int vCol)
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

template <typename Scalar_t, unsigned Channel>
Eigen::Matrix<float, -1, -1> CMipmapGenerator<Scalar_t, Channel>::__getGaussianKernal(int vKernalSize, float vSigma)
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

template <typename Scalar_t, unsigned Channel>
float CMipmapGenerator<Scalar_t, Channel>::__getGaussianWeight(float vRadius, float vSigma)
{
	// Considering that normalization is required later, the coefficient is not calculated
	return std::pow(std::numbers::e, -vRadius * vRadius / (2 * vSigma * vSigma));
}
