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

template <typename Color_t>
CMipmapGenerator<Color_t>::Texture_t CMipmapGenerator<Color_t>::getMipmap(const Texture_t& vTexture)
{
	Texture_t Mipmap((vTexture.rows() + 1) / 2, (vTexture.cols() + 1) / 2);
	executeGaussianBlur(vTexture, Mipmap);
	return Mipmap;
}

template <typename Color_t>
void CMipmapGenerator<Color_t>::executeGaussianBlur(const Texture_t &vTexture, Texture_t &voMipmap)
{
	auto GaussianKernal = __getGaussianKernal(m_KernalSize, 0);

	for (int i = 0; i < voMipmap.rows(); i++)
		for (int k = 0; k < voMipmap.cols(); k++)
			voMipmap(i, k) = __executeGaussianFilter(vTexture, GaussianKernal, i * 2, k * 2);
}

template <typename Color_t>
CMipmapGenerator<Color_t>::Texture_t CMipmapGenerator<Color_t>::executeGaussianBlur(const Texture_t& vTexture)
{
	Texture_t ResultTexture(vTexture.rows(), vTexture.cols());
	auto GaussianKernal = __getGaussianKernal(m_KernalSize, 0);

	for (int i = 0; i < ResultTexture.rows(); i++)
		for (int k = 0; k < ResultTexture.cols(); k++)
			ResultTexture(i, k) = __executeGaussianFilter(vTexture, GaussianKernal, i, k);
	return ResultTexture;
}

template <typename Color_t>
Color_t CMipmapGenerator<Color_t>::__executeGaussianFilter(const Texture_t &vTexture, const Eigen::Matrix<float, -1, -1> &vGaussianKernal, int vRow, int vCol)
{
	Color_t Value = vTexture.coeff(0, 0);
	Value -= Value;
	Eigen::Vector3f Valuef = { 0.0f, 0.0f, 0.0f };

	int GaussianKernalRowIndex = -1;
	int GaussianKernalColIndex = -1;
	float GaussianWeightSum = 0.0;
	for (int i = vRow - (vGaussianKernal.rows() - 1) / 2; i <= vRow + (vGaussianKernal.rows() - 1) / 2; i++)
	{
		GaussianKernalRowIndex++;
		for (int k = vCol - (vGaussianKernal.cols() - 1) / 2; k <= vCol + (vGaussianKernal.cols() - 1) / 2; k++)
		{
			GaussianKernalColIndex++;
			if (i < 0 || k < 0 || i >= vTexture.rows() || k >= vTexture.cols())
				continue;
			auto Weight = vGaussianKernal.coeff(GaussianKernalRowIndex, GaussianKernalColIndex);
			if constexpr (std::is_arithmetic<Color_t>::value)
				Value += Weight * vTexture.coeff(i, k);
			else
				for (int m = 0; m < 3; m++)
					Valuef[m] += Weight * vTexture.coeff(i, k)[m];
			GaussianWeightSum += Weight;
		}
		GaussianKernalColIndex = -1;
	}

	if constexpr (std::is_arithmetic<Color_t>::value)
		Value /= GaussianWeightSum;
	else
	{
		Valuef /= GaussianWeightSum;
		for (int i = 0; i < Valuef.size(); i++)
			Value[i] = int(std::round(Valuef[i]));
	}
		
	return Value;
}

template <typename Color_t>
Eigen::Matrix<float, -1, -1> CMipmapGenerator<Color_t>::__getGaussianKernal(int vKernalSize, float vSigma)
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

template <typename Color_t>
float CMipmapGenerator<Color_t>::__getGaussianWeight(float vRadius, float vSigma)
{
	// Considering that normalization is required later, the coefficient is not calculated
	return std::pow(std::numbers::e, -vRadius * vRadius / (2 * vSigma * vSigma));
}

template <typename Color_t>
void CMipmapGenerator<Color_t>::setKernalSize(int vKernalSize)
{
	m_KernalSize = vKernalSize;
}

template <typename Color_t>
auto CMipmapGenerator<Color_t>::getGaussianPyramid(const Texture_t& vTexture, int vLayer) -> std::vector<CMipmapGenerator<Color_t>::Texture_t>
{

}