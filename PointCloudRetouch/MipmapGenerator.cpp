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
CMipmapGenerator<Color_t>::Texture_t CMipmapGenerator<Color_t>::getMipmap(const Texture_t& vTexture, const Eigen::MatrixXi& vMask)
{
	Texture_t TextureWithMask = vTexture;

	const float CoverRate = 0.75f;	//¸²¸ÇÂÊ
	std::pair<float, float> SampleRate{ (float)vMask.rows() / vTexture.rows(), (float)vMask.cols() / vTexture.cols() };
	for (int i = 0; i < TextureWithMask.rows(); i++)
		for (int k = 0; k < TextureWithMask.cols(); k++)
		{
			int Sum = 0;
			for (int MaskY = i * SampleRate.first; MaskY < (i + 1) * SampleRate.first; MaskY++)
				for (int MaskX = k * SampleRate.second; MaskX < (k + 1) * SampleRate.second; MaskX++)
					if (MaskX < vMask.cols() && MaskY < vMask.rows() && vMask(MaskY, MaskX))
						Sum++;

			if (Sum >= CoverRate * SampleRate.first * SampleRate.second)
			{
				if constexpr (std::is_arithmetic<Color_t>::value)
					TextureWithMask(i, k) = -1;
				else
					TextureWithMask(i, k)[0] = -1;
			}
		}

	Texture_t Mipmap((TextureWithMask.rows() + 1) / 2, (TextureWithMask.cols() + 1) / 2);
	executeGaussianBlur(TextureWithMask, vMask, Mipmap);
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
void CMipmapGenerator<Color_t>::executeGaussianBlur(const Texture_t& vTexture, const Eigen::MatrixXi& vMask, Texture_t& voMipmap)
{
	auto GaussianKernal = __getGaussianKernal(m_KernalSize, 0);

	for (int i = 0; i < voMipmap.rows(); i++)
		for (int k = 0; k < voMipmap.cols(); k++)
			voMipmap(i, k) = __executeGaussianFilter(vTexture, GaussianKernal, i * 2, k * 2);

	const float CoverRate = 0.75f;
	std::pair<float, float> SampleRate{ (float)vMask.rows() / voMipmap.rows(), (float)vMask.cols() / voMipmap.cols() };
	for (int i = 0; i < voMipmap.rows(); i++)
		for (int k = 0; k < voMipmap.cols(); k++)
		{
			int Sum = 0;
			for (int MaskY = i * SampleRate.first; MaskY < (i + 1) * SampleRate.first; MaskY++)
				for (int MaskX = k * SampleRate.second; MaskX < (k + 1) * SampleRate.second; MaskX++)
					if (MaskX < vMask.cols() && MaskY < vMask.rows() && vMask(MaskY, MaskX))
						Sum++;

			if (Sum >= CoverRate * SampleRate.first * SampleRate.second)
				if constexpr (std::is_arithmetic<Color_t>::value)
					voMipmap(i, k) = -1;
				else
					voMipmap(i, k)[0] = -1;
		}
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
CMipmapGenerator<Color_t>::Texture_t CMipmapGenerator<Color_t>::executeGaussianBlur(const Texture_t& vTexture, const Eigen::MatrixXi& vMask)
{
	Texture_t ResultTexture(vTexture.rows(), vTexture.cols());
	auto GaussianKernal = __getGaussianKernal(m_KernalSize, 0);

	for (int i = 0; i < ResultTexture.rows(); i++)
		for (int k = 0; k < ResultTexture.cols(); k++)
			ResultTexture(i, k) = __executeGaussianFilter(vTexture, GaussianKernal, i, k);

	const float CoverRate = 0.75f;
	std::pair<float, float> SampleRate{ (float)vMask.rows() / ResultTexture.rows(), (float)vMask.cols() / ResultTexture.cols() };
	for (int i = 0; i < ResultTexture.rows(); i++)
		for (int k = 0; k < ResultTexture.cols(); k++)
		{
			int Sum = 0;
			for (int MaskY = i * SampleRate.first; MaskY < (i + 1) * SampleRate.first; MaskY++)
				for (int MaskX = k * SampleRate.second; MaskX < (k + 1) * SampleRate.second; MaskX++)
					if (MaskX < vMask.cols() && MaskY < vMask.rows() && vMask(MaskY, MaskX))
						Sum++;

			if (Sum >= CoverRate * SampleRate.first * SampleRate.second)
				if constexpr (std::is_arithmetic<Color_t>::value)
					ResultTexture(i, k) = -1;
				else
					ResultTexture(i, k)[0] = -1;
		}

	return ResultTexture;
}

template <typename Color_t>
Color_t CMipmapGenerator<Color_t>::__executeGaussianFilter(const Texture_t &vTexture, const Eigen::Matrix<float, -1, -1> &vGaussianKernal, int vRow, int vCol)
{
	Color_t Value = vTexture.coeff(0, 0);
	Value -= Value;
	Eigen::Vector3f Valuef = { 0.0f, 0.0f, 0.0f };
	Eigen::MatrixXd Flags = Eigen::MatrixXd::Zero(vGaussianKernal.rows(), vGaussianKernal.cols());

	int GaussianKernalRowIndex = -1;
	int GaussianKernalColIndex = -1;
	float GaussianWeightSum = 0.0;
	double IfFlag = 0.0;
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
			{
				if (vTexture.coeff(i, k) < 0)
					continue;
				Value += Weight * vTexture.coeff(i, k);
			}
			else
			{
				if (vTexture.coeff(i, k)[0] < 0)
					continue;
				for (int m = 0; m < 3; m++)
					Valuef[m] += Weight * vTexture.coeff(i, k)[m];
			}
			Flags(GaussianKernalRowIndex, GaussianKernalColIndex) = 1;
			GaussianWeightSum += Weight;
		}
		GaussianKernalColIndex = -1;
	}

	for (int i = 0; i < Flags.rows(); i++)
		for (int k = 0; k < Flags.cols(); k++)
			IfFlag += Flags.coeff(i, k);
			
	if (!IfFlag)
	{
		if constexpr (std::is_arithmetic<Color_t>::value)
			return -1;
		else
			for (int m = 0; m < 3; m++)
				Value[m] = -1;
		return Value;
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
void CMipmapGenerator<Color_t>::setKernalSize(const int vKernalSize)
{
	m_KernalSize = vKernalSize;
}

template <typename Color_t>
auto CMipmapGenerator<Color_t>::getGaussianPyramid(const Texture_t& vTexture, const int vLayer) -> std::vector<Texture_t>
{
	std::vector<Texture_t> GaussianPyramid;
	GaussianPyramid.push_back(vTexture);

	for (int i = 0; i < vLayer - 1; i++)
		GaussianPyramid.push_back(getMipmap(GaussianPyramid.back()));

	std::reverse(std::begin(GaussianPyramid), std::end(GaussianPyramid));
	return GaussianPyramid;
}

template <typename Color_t>
auto CMipmapGenerator<Color_t>::getGaussianPyramid(const Texture_t& vTexture, const int vLayer, const Eigen::MatrixXi& vMask) -> std::vector<Texture_t>
{
	Texture_t TextureWithMask = vTexture;
	for (int i = 0; i < TextureWithMask.rows(); i++)
		for (int k = 0; k < TextureWithMask.cols(); k++)
			if (vMask.coeff(i, k))
			{
				if constexpr (std::is_arithmetic<Color_t>::value)
					TextureWithMask(i, k) = -1;
				else
					for (int m = 0; m < 3; m++)
						TextureWithMask(i, k)[m] = -1;
			}

	std::vector<Texture_t> GaussianPyramid;
	GaussianPyramid.push_back(TextureWithMask);

	for (int i = 0; i < vLayer - 1; i++)
		GaussianPyramid.push_back(getMipmap(GaussianPyramid.back(), vMask));

	std::reverse(std::begin(GaussianPyramid), std::end(GaussianPyramid));
	return GaussianPyramid;
}

template <typename Color_t>
auto CMipmapGenerator<Color_t>::getGaussianStack(const Texture_t& vTexture, const int vLayer) -> std::vector<Texture_t>
{
	std::vector<Texture_t> GaussianStack;
	GaussianStack.push_back(vTexture);

	for (int i = 0; i < vLayer - 1; i++)
		GaussianStack.push_back(executeGaussianBlur(GaussianStack.back()));

	std::reverse(std::begin(GaussianStack), std::end(GaussianStack));
	return GaussianStack;
}

template <typename Color_t>
auto CMipmapGenerator<Color_t>::getGaussianStack(const Texture_t& vTexture, const int vLayer, const Eigen::MatrixXi& vMask) -> std::vector<Texture_t>
{
	Texture_t TextureWithMask = vTexture;
	for (int i = 0; i < TextureWithMask.rows(); i++)
		for (int k = 0; k < TextureWithMask.cols(); k++)
			if (vMask.coeff(i, k))
			{
				if constexpr (std::is_arithmetic<Color_t>::value)
					TextureWithMask(i, k) = -1;
				else
					for (int m = 0; m < 3; m++)
						TextureWithMask(i, k)[m] = -1;
			}

	std::vector<Texture_t> GaussianStack;
	GaussianStack.push_back(TextureWithMask);

	for (int i = 0; i < vLayer - 1; i++)
		GaussianStack.push_back(executeGaussianBlur(GaussianStack.back()));

	std::reverse(std::begin(GaussianStack), std::end(GaussianStack));
	return GaussianStack;
}