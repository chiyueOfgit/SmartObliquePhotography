#pragma once

namespace hiveObliquePhotography::PointCloudRetouch
{
	using Scalar_t = int;
	constexpr unsigned Channel = 3;
	//template <typename Scalar_t, unsigned Channel>
	class COrderIndependentTextureSynthesizer
	{
	public:
		using Texture_t = Eigen::Matrix<Eigen::Matrix<Scalar_t, Channel, 1>, Eigen::Dynamic, Eigen::Dynamic>;
		COrderIndependentTextureSynthesizer() = default;
		~COrderIndependentTextureSynthesizer() = default;

		void execute(const Texture_t& vInput, const Eigen::MatrixXi& vMask, Texture_t& vioScene);

	private:
		using Feature_t = Eigen::Matrix<Scalar_t, Eigen::Dynamic, 1>;
		using NeighborOffset_t = std::vector<std::pair<int, int>>;
		//TODO: magic number
		int m_KernelSize = 7;
		int m_GaussianSize = 7;
		int m_PyramidLayer = 4;
		int m_GenerationNum = 3;
		NeighborOffset_t m_NeighborOffset;
		std::vector<Texture_t> m_InputPyramid;
		std::vector<std::vector<Texture_t>> m_Cache;

		static Scalar_t __computeDistance(const Feature_t& vLhs, const Feature_t& vRhs) { return (vLhs - vRhs).squaredNorm(); }
		static bool __isAvailable(const Texture_t::value_type& vValue) { return (vValue.array() >= 0).all(); }
		static void __wrap(Eigen::Index vSize, Eigen::Index& vioIndex) { while (vioIndex < 0) vioIndex += vSize; while (vioIndex >= vSize) vioIndex -= vSize; }
		static NeighborOffset_t __buildNeighborOffset(int vKernelSize);

		void __initCache(const Eigen::MatrixXi& vMask, const Texture_t& vOutput);
		void __initInputPyramid(const Texture_t& vTexture);
		void __initTexture(const Texture_t& vFrom, Texture_t& voTo) const;

		void __decrease(int& vioLayer, int& vioGeneration, Eigen::Index& vioRowId, Eigen::Index& vioColId) const;
		Texture_t::value_type __getInputAt(int vLayer, Eigen::Index vRowId, Eigen::Index vColId) const;
		Texture_t::value_type __getCacheAt(int vLayer, int vGeneration, Eigen::Index vRowId, Eigen::Index vColId) const;
		void __addCacheEntry(int vLayer, int vGeneration, Eigen::Index vRowId, Eigen::Index vColId, const Texture_t::value_type& vValue);
		
		Texture_t::value_type __synthesizePixel(int vLayer, int vGeneration, Eigen::Index vRowId, Eigen::Index vColId);
		Feature_t __buildOutputFeatureAt(int vLayer, int vGeneration, Eigen::Index vRowId, Eigen::Index vColId);
		Feature_t __buildInputFeatureAt(int vLayer, int vGeneration, Eigen::Index vRowId, Eigen::Index vColId) const;
		
		Texture_t::value_type __findNearestValue(int vLayer, int vGeneration, const Feature_t& vFeature) const;

		void generateResultImage(const Eigen::Matrix<Eigen::Vector3i, -1, -1>& vTexture, const std::string& vOutputImagePath);

	};

	//template class COrderIndependentTextureSynthesizer<int, 3>;
	//template class COrderIndependentTextureSynthesizer<float, 1>;
}
