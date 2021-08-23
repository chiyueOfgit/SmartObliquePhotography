#pragma once
#include <flann/flann.hpp>

namespace hiveObliquePhotography::PointCloudRetouch
{
	//template <typename Scalar_t, unsigned Channel>
	using Scalar_t = int;
	extern constexpr int Channel = 3;
	class CMultithreadTextureSynthesizer
	{
	public:
		using Texture_t = Eigen::Matrix<Eigen::Matrix<Scalar_t, Channel, 1>, Eigen::Dynamic, Eigen::Dynamic>;
		CMultithreadTextureSynthesizer() = default;
		~CMultithreadTextureSynthesizer() = default;

		void execute(const Texture_t& vInput, const Eigen::MatrixXi& vMask, Texture_t& vioScene);

	private:
		using Color_t = Eigen::Matrix<Scalar_t, Channel, 1>;
		using Feature_t = Eigen::Matrix<Scalar_t, 1, Eigen::Dynamic>;
		//TODO: magic number
		int m_KernelSize = 9;
		int m_GaussianSize = 9;
		int m_PyramidLayer = 3;
		int m_GenerationNum = 4;
		std::vector<std::pair<int, int>> m_NeighborOffset;
		std::vector<Texture_t> m_InputPyramid;
		std::vector<std::vector<Texture_t>> m_Cache;

		static auto __computeDistance(const Feature_t& vLhs, const Feature_t& vRhs) { return (vLhs - vRhs).squaredNorm(); }
		static bool __isAvailable(const Color_t& vValue) { return (vValue.array() >= 0).all(); }
		static Eigen::Index __wrap(Eigen::Index vSize, Eigen::Index vIndex)
		{
			//if (vIndex < 0) vIndex = 0;
			//else if (vIndex >= vSize) vIndex = vSize - 1;
			//return vIndex;
			if (vIndex < 0) vIndex += vSize;
			else if (vIndex >= vSize) vIndex -= vSize;
			return vIndex;
		}
		static std::vector<std::pair<int, int>> __buildNeighborOffset(int vKernelSize);

		void __initCache(const Eigen::MatrixXi& vMask, const Texture_t& vOutput);
		void __initInputPyramid(const Texture_t& vTexture);
		void __initTexture(const Texture_t& vFrom, Texture_t& voTo) const;
		void __initTextureWithNeighborMask(const Texture_t& vFrom, Texture_t& voTo) const;

		bool __increase(int& vioLayer, int& vioGeneration) const;
		bool __decrease(int& vioLayer, int& vioGeneration) const;

		void __synthesizeTexture(int vLayer, int vGeneration);
		Color_t __findNearestValue(int vLayer, int vGeneration, const Feature_t& vFeature) const;

		Feature_t __buildInputFeatureAt(int vLayer, int vGeneration, Eigen::Index vRowId, Eigen::Index vColId) const;
		Feature_t __buildOutputFeatureAt(int vLayer, int vGeneration, Eigen::Index vRowId, Eigen::Index vColId) const;
		Feature_t __buildFeatureAt(const Texture_t& vTexture, Eigen::Index vRowId, Eigen::Index vColId) const;
		Feature_t __buildFeatureWithNeighborMask(const Texture_t& vTexture, Eigen::Index vRowId, Eigen::Index vColId, const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& vMask) const;

		void __generateResultImage(const Texture_t& vTexture, const std::string& vOutputImagePath) const;
	};

	//template class CMultithreadTextureSynthesizer<int, 3>;
	//template class CMultithreadTextureSynthesizer<float, 1>;
}
