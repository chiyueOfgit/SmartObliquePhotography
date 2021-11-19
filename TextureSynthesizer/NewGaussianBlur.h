#pragma once
#include <numbers>
#include <Eigen/Eigen>

namespace HiveTextureSynthesizer
{
	template <typename TColor>
	concept gaussianblur_color = std::is_integral<TColor>::value || std::is_same_v<TColor, float> || std::is_same_v<TColor, Eigen::Vector3i> ||
		std::is_same_v<TColor, Eigen::Matrix<int, 3, 1>> || std::is_same_v<TColor, Eigen::Matrix<float, 1, 1>>;

	template <gaussianblur_color TColor>
	class CGaussianBlur
	{
	public:
		using Texture_t = Eigen::Matrix<TColor, Eigen::Dynamic, Eigen::Dynamic>;
		CGaussianBlur() = default;
		~CGaussianBlur() = default;

		void executeColorMix(const Texture_t& vTexture, Texture_t& voMixedTexture);
		void executeGaussianBlur(const Texture_t& vTexture, Texture_t& voBluredTexture);
		Texture_t executeGaussianBlur(const Texture_t& vTexture);
		void setKernelSize(const int vKernalSize);

	private:
		float __getGaussianWeight(float vRadius, float vSigma);
		Eigen::Matrix<float, -1, -1> __computeGaussianKernel(int vKernalSize, float vSigma);
		TColor __executeGaussianFilter(const Texture_t& vTexture, const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& vGaussianKernal, int vRow, int vCol);
		TColor __executeMix(const Texture_t& vTexture, int vRow, int vCol);

		int m_KernelSize = 5;
	};

	template <gaussianblur_color TColor>
	void CGaussianBlur<TColor>::executeGaussianBlur(const Texture_t& vTexture, Texture_t& voBluredTexture)
	{
		_ASSERTE(vTexture.rows() > 0 && vTexture.cols() > 0);
		auto GaussianKernal = __computeGaussianKernel(m_KernelSize, 0);

		for (int i = 0; i < voBluredTexture.rows(); i++)
			for (int k = 0; k < voBluredTexture.cols(); k++)
				voBluredTexture(i, k) = __executeGaussianFilter(vTexture, GaussianKernal, i * 2, k * 2);
	}

	template <gaussianblur_color TColor>
	void CGaussianBlur<TColor>::executeColorMix(const Texture_t& vTexture, Texture_t& voMixedTexture)
	{
		_ASSERTE(vTexture.rows() > 0 && vTexture.cols() > 0);

		for (int i = 0; i < voMixedTexture.rows(); i++)
			for (int k = 0; k < voMixedTexture.cols(); k++)
				voMixedTexture(i, k) = __executeMix(vTexture, i, k);
	}

	template <gaussianblur_color TColor>
	TColor CGaussianBlur<TColor>::__executeMix(const Texture_t& vTexture, int vRow, int vCol)
	{
		std::vector<Eigen::Vector2i> CorrespondingCoord{ {2 * vRow,2 * vCol}, {2 * vRow + 1,2 * vCol}, {2 * vRow,2 * vCol + 1},{2 * vRow + 1,2 * vCol + 1} };
		TColor SumColor = 0;
		int ValidNum = 0;
		for (auto& Coord : CorrespondingCoord)
		{
			if (vTexture.coeff(Coord.x(), Coord.y()) > 0)
			{
				SumColor += vTexture.coeff(Coord.x(), Coord.y());
				ValidNum++;
			}
		}
		if (ValidNum == 0)
			return SumColor;
		SumColor /= ValidNum;
		return SumColor;
	}
	
	template <gaussianblur_color TColor>
	CGaussianBlur<TColor>::Texture_t CGaussianBlur<TColor>::executeGaussianBlur(const Texture_t& vTexture)
	{
		Texture_t ResulTexture_t(vTexture.rows(), vTexture.cols());
		auto GaussianKernal = __computeGaussianKernel(m_KernelSize, 0);

		for (int i = 0; i < ResulTexture_t.rows(); i++)
			for (int k = 0; k < ResulTexture_t.cols(); k++)
				ResulTexture_t(i, k) = __executeGaussianFilter(vTexture, GaussianKernal, i, k);
		return ResulTexture_t;
	}

	template <gaussianblur_color TColor>
	TColor CGaussianBlur<TColor>::__executeGaussianFilter(const Texture_t& vTexture, const Eigen::Matrix<float, -1, -1>& vGaussianKernal, int vRow, int vCol)
	{
		TColor Value = vTexture.coeff(0, 0);
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
				if (i < 0 || k < 0 || i >= vTexture.rows() || k >= vTexture.cols()) continue;
				auto Weight = vGaussianKernal.coeff(GaussianKernalRowIndex, GaussianKernalColIndex);
				if constexpr (std::is_arithmetic<TColor>::value)
				{
					if (vTexture.coeff(i, k) < 0) continue;
					Value += Weight * vTexture.coeff(i, k);
				}
				else
				{
					if (vTexture.coeff(i, k)[0] < 0) continue;
					for (int m = 0; m < TColor::MaxRowsAtCompileTime; m++)
						Valuef[m] += Weight * vTexture.coeff(i, k)[m];
				}
				GaussianWeightSum += Weight;
			}
			GaussianKernalColIndex = -1;
		}

		if constexpr (std::is_arithmetic<TColor>::value)
			Value = (GaussianWeightSum == 0) ? -1 : Value / GaussianWeightSum;
		else
		{
			if (GaussianWeightSum == 0)
				for (int m = 0; m < TColor::MaxRowsAtCompileTime; m++)
					Value[m] = -1;
			else
			{
				Valuef /= GaussianWeightSum;
				for (int i = 0; i < Valuef.size(); i++)
					Value[i] = int(std::round(Valuef[i]));
			}
		}

		return Value;
	}

	template <gaussianblur_color TColor>
	Eigen::Matrix<float, -1, -1> CGaussianBlur<TColor>::__computeGaussianKernel(int vKernelSize, float vSigma)
	{
		if (!vSigma)
			vSigma = 0.3 * ((vKernelSize - 1) * 0.5 - 1) + 0.8;

		float SumWeight = 0.0;
		Eigen::Matrix<float, -1, -1> GaussianKernal(vKernelSize, vKernelSize);

		for (int i = 0; i < vKernelSize; i++)
			for (int k = 0; k < vKernelSize; k++)
			{
				GaussianKernal(i, k) = __getGaussianWeight(std::abs((vKernelSize - 1) / 2 - i) + std::abs((vKernelSize - 1) / 2 - k), vSigma);
				SumWeight += GaussianKernal.coeff(i, k);
			}

		GaussianKernal /= SumWeight;

		return GaussianKernal;
	}

	template <gaussianblur_color TColor>
	float CGaussianBlur<TColor>::__getGaussianWeight(float vRadius, float vSigma)
	{
		// Considering that normalization is required later, the coefficient is not calculated
		return std::pow(std::numbers::e, -vRadius * vRadius / (2 * vSigma * vSigma));
	}

	template <gaussianblur_color TColor>
	void CGaussianBlur<TColor>::setKernelSize(const int vKernelSize)
	{
		m_KernelSize = vKernelSize;
	}
}