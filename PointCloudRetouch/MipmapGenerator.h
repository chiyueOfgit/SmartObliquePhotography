#pragma once

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch {

		template <typename Scalar_t, unsigned Channel>
		class CMipmapGenerator
		{
		public:
			using Texture_t = Eigen::Matrix<Eigen::Matrix<Scalar_t, Channel, 1>, Eigen::Dynamic, Eigen::Dynamic>;
			CMipmapGenerator() = default;
			~CMipmapGenerator() = default;

			Texture_t getMipmap(const Texture_t& vTexture);

		private:
			float __getGaussianWeight(float vRadius, float vSigma);
			Eigen::Matrix<float, -1, -1> __getGaussianKernal(int vKernalSize, float vSigma);
			float __executeGaussianFilter(const Texture_t& vTexture, const Eigen::Matrix<float, -1, -1>& vGaussianKernal, int vChannel, int vRow, int vCol);
			void __executeGaussianBlur(const Texture_t& vTexture, Texture_t& voMipmap);
		};

		template class CMipmapGenerator<int, 3>;
		template class CMipmapGenerator<float, 1>;

	}
}

