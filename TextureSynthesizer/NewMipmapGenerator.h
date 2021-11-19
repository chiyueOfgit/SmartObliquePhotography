#pragma once
#include <Eigen/Eigen>
#include "NewGaussianBlur.h"

namespace HiveTextureSynthesizer
{
	template <typename TColor>
	concept mipmap_color = std::is_integral<TColor>::value || std::is_same_v<TColor, float> || std::is_same_v<TColor, Eigen::Vector3i> ||
		std::is_same_v<TColor, Eigen::Matrix<int, 3, 1>> || std::is_same_v<TColor, Eigen::Matrix<float, 1, 1>>;

	template <mipmap_color TColor>
	class CNewMipmapGenerator
	{
	public:
		using Texture_t = Eigen::Matrix<TColor, Eigen::Dynamic, Eigen::Dynamic>;
		CNewMipmapGenerator() = default;
		~CNewMipmapGenerator() = default;
		std::vector<Texture_t> getMipmapStack(const Texture_t& vTexture, const int vLayer);
		std::vector<Texture_t> computeMipmapPyramid(const Texture_t& vTexture, const int vTotalLayer);
		Texture_t downsampleTexture(const Texture_t& vTexture);
		Texture_t downsampleTexture4New(const Texture_t& vTexture);
		void setKernalSize(const int vKernalSize) { m_KernalSize = vKernalSize; }

	private:
		int m_KernalSize = 9;
	};

	template <mipmap_color TColor>
	CNewMipmapGenerator<TColor>::Texture_t CNewMipmapGenerator<TColor>::downsampleTexture(const Texture_t& vTexture)
	{
		//TODO: 分离模糊和缩小
		Texture_t DownsampledTexture((vTexture.rows() + 1) / 2, (vTexture.cols() + 1) / 2);
		CGaussianBlur<TColor> GaussianBlur;
		GaussianBlur.executeGaussianBlur(vTexture, DownsampledTexture);
		return DownsampledTexture;
	}

	template <mipmap_color TColor>
	CNewMipmapGenerator<TColor>::Texture_t CNewMipmapGenerator<TColor>::downsampleTexture4New(const Texture_t& vTexture)
	{
		//TODO: 分离模糊和缩小
		Texture_t DownsampledTexture((vTexture.rows() + 1) / 2, (vTexture.cols() + 1) / 2);
		CGaussianBlur<TColor> GaussianBlur;
		GaussianBlur.executeColorMix(vTexture, DownsampledTexture);
		return DownsampledTexture;
	}
	
	template <mipmap_color TColor>
	std::vector<Eigen::Matrix<TColor, Eigen::Dynamic, Eigen::Dynamic>> CNewMipmapGenerator<TColor>::computeMipmapPyramid(const Texture_t& vTexture, int vTotalLayer)
	{
		if (vTotalLayer <= 1)
			_THROW_RUNTIME_ERROR("Error: Illegal mipmap total layer");
		std::vector<Texture_t> GaussianPyramid(1, vTexture);
		for (int i = 0; i < vTotalLayer - 1; i++)
			GaussianPyramid.push_back(downsampleTexture(GaussianPyramid.back()));

		std::reverse(std::begin(GaussianPyramid), std::end(GaussianPyramid));
		return GaussianPyramid;
	}

	template <mipmap_color TColor>
	std::vector<Eigen::Matrix<TColor, Eigen::Dynamic, Eigen::Dynamic>> CNewMipmapGenerator<TColor>::getMipmapStack(const Texture_t& vTexture, const int vLayer)
	{
		std::vector<Texture_t> MipmapStack;
		MipmapStack.push_back(vTexture);

		CGaussianBlur<TColor> GaussianBlur;
		for (int i = 0; i < vLayer - 1; i++)
			MipmapStack.push_back(GaussianBlur.executeGaussianBlur(MipmapStack.back()));

		std::reverse(std::begin(MipmapStack), std::end(MipmapStack));
		return MipmapStack;
	}
}

