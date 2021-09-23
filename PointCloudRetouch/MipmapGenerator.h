#pragma once

namespace hiveObliquePhotography::PointCloudRetouch
{
	template <typename Color_t>
	class CMipmapGenerator
	{
	public:
		using Texture_t = Eigen::Matrix<Color_t, Eigen::Dynamic, Eigen::Dynamic>;
		CMipmapGenerator() = default;
		~CMipmapGenerator() = default;

		std::vector<Texture_t> getGaussianStack(const Texture_t& vTexture, const int vLayer);
		std::vector<Texture_t> getGaussianStack(const Texture_t& vTexture, const int vLayer, const Eigen::MatrixXi& vMask);
		std::vector<Texture_t> getGaussianPyramid(const Texture_t& vTexture, const int vLayer);
		std::vector<Texture_t> getGaussianPyramid(const Texture_t& vTexture, const int vLayer, const Eigen::MatrixXi& vMask);
		Texture_t getMipmap(const Texture_t& vTexture);
		Texture_t getMipmap(const Texture_t& vTexture, const Eigen::MatrixXi& vMask);
		void executeGaussianBlur(const Texture_t& vTexture, Texture_t& voMipmap);
		void executeGaussianBlur(const Texture_t& vTexture, const Eigen::MatrixXi& vMask, Texture_t& voMipmap);
		Texture_t executeGaussianBlur(const Texture_t& vTexture);
		Texture_t executeGaussianBlur(const Texture_t& vTexture, const Eigen::MatrixXi& vMask);
		void setKernalSize(const int vKernalSize);

	private:
		float __getGaussianWeight(float vRadius, float vSigma);
		Eigen::Matrix<float, -1, -1> __getGaussianKernal(int vKernalSize, float vSigma);
		Color_t __executeGaussianFilter(const Texture_t& vTexture, const Eigen::Matrix<float, -1, -1>& vGaussianKernal, int vRow, int vCol);

		void __fixTextureByMask(Texture_t& vTexture, const Eigen::MatrixXi& vMask);

		// TODO::magic number
		int m_KernalSize = 9;
	};

	template class CMipmapGenerator<float>;
	template class CMipmapGenerator<int>;
	template class CMipmapGenerator<Eigen::Vector3i>;
	template class CMipmapGenerator<Eigen::Matrix<int, 3, 1>>;
	template class CMipmapGenerator<Eigen::Matrix<float, 1, 1>>;
}

