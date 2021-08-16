#pragma once
#include <flann/flann.hpp>

namespace hiveObliquePhotography::PointCloudRetouch
{
	//using Scalar_t = int;
	//constexpr int Channel = 3;

	template <typename Scalar_t, unsigned Channel>
	class CTreeBasedTextureSynthesizer
	{
	public:
		using Texture_t = Eigen::Matrix<Eigen::Matrix<Scalar_t, Channel, 1>, Eigen::Dynamic, Eigen::Dynamic>;
		CTreeBasedTextureSynthesizer() = default;
		~CTreeBasedTextureSynthesizer() = default;

		void execute(const Texture_t& vInput, const Eigen::MatrixXi& vMask, Texture_t& vioScene);

	private:
		using Color_t = Texture_t::value_type;
		using Feature_t = Eigen::Matrix<Scalar_t, 1, Eigen::Dynamic>;
		//TODO: magic number
		int m_KernelSize = 9;
		int m_GaussianSize = 9;
		int m_PyramidLayer = 4;
		int m_GenerationNum = 3;
		std::vector<std::pair<int, int>> m_NeighborOffset;
		std::vector<std::tuple<flann::Index<flann::L2<Scalar_t>>* , Eigen::Matrix<Scalar_t, -1, -1, Eigen::RowMajor>, Eigen::Matrix<Scalar_t, -1, Channel>, Eigen::Matrix<Scalar_t, -1, Channel>>> m_SearchSet;
		std::vector<std::vector<Texture_t>> m_Cache;

		static auto __computeDistance(const Feature_t& vLhs, const Feature_t& vRhs) { return (vLhs - vRhs).squaredNorm(); }
		static bool __isAvailable(const Color_t& vValue) { return (vValue.array() >= 0).all(); }
		static Eigen::Index __wrap(Eigen::Index vSize, Eigen::Index vIndex)
		{
			if (vIndex < 0) vIndex += vSize;
			else if (vIndex >= vSize) vIndex -= vSize;
			return vIndex;
		}
		static std::vector<std::pair<int, int>> __buildNeighborOffset(int vKernelSize);

		void __initCache(const Eigen::MatrixXi& vMask, const Texture_t& vOutput);
		void __initSearchSet(const Texture_t& vTexture, int vFeatureLength);
		void __initTexture(const Texture_t& vFrom, Texture_t& voTo) const;
		void __initTextureWithNeighborMask(const Texture_t& vFrom, Texture_t& voTo) const;

		bool __increase(int& vioLayer, int& vioGeneration) const;
		bool __decrease(int& vioLayer, int& vioGeneration) const;

		void __synthesizeTexture(int vLayer, int vGeneration);
		Color_t __findNearestValue(int vLayer, int vGeneration, const Feature_t& vFeature) const;
		
		Feature_t __buildOutputFeatureAt(int vLayer, int vGeneration, Eigen::Index vRowId, Eigen::Index vColId) const;
		Feature_t __buildFeatureAt(const Texture_t& vTexture, Eigen::Index vRowId, Eigen::Index vColId) const;
		Feature_t __buildFeatureWithNeighborMask(const Texture_t& vTexture, Eigen::Index vRowId, Eigen::Index vColId, const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& vMask) const;

		void __generateResultImage(const Texture_t& vTexture, const std::string& vOutputImagePath) const;
	};

	template class CTreeBasedTextureSynthesizer<int, 3>;
	template class CTreeBasedTextureSynthesizer<float, 1>;
}
