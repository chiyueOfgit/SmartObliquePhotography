#pragma once

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch {

		template <typename Color_t>
		class CMipmapGenerator
		{
		public:
			using Texture_t = Eigen::Matrix<Color_t, Eigen::Dynamic, Eigen::Dynamic>;
			CMipmapGenerator() = default;
			~CMipmapGenerator() = default;

			Texture_t getMipmap(const Texture_t& vTexture);

		private:
			float __getGaussianWeight(float vRadius, float vSigma);
			Eigen::Matrix<float, -1, -1> __getGaussianKernal(int vKernalSize, float vSigma);
			Color_t __executeGaussianFilter(const Texture_t& vTexture, const Eigen::Matrix<float, -1, -1>& vGaussianKernal, int vRow, int vCol);
			void __executeGaussianBlur(const Texture_t& vTexture, Texture_t& voMipmap);
		};

		template class CMipmapGenerator<float>;
		template class CMipmapGenerator<int>;
		template class CMipmapGenerator<Eigen::Vector3i>;

	}
}

