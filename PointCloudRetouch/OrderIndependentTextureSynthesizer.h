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
		using Feature_t = std::vector<Eigen::Matrix<Scalar_t, Channel, 1>>;
		//TODO: magic number
		int m_KernelSize = 9;
		int m_PyramidLayer = 3;
		std::vector<std::pair<int, int>> m_NeighborOffset;
		std::vector<std::tuple<Texture_t, Eigen::MatrixXi, Texture_t>> m_Pyramid;
		
		static Scalar_t __computeDistance(const Feature_t& vLhs, const Feature_t& vRhs);
		static std::vector<std::pair<int, int>> __buildNeighborOffset(int vKernelSize);
		void __buildPyramid(const Texture_t& vInput, const Eigen::MatrixXi& vMask, const Texture_t& vOutput);
		Feature_t __generateFeatureAt(const Texture_t& vTexture, size_t vRowId, size_t vColId) const;
		std::pair<Eigen::Index, Eigen::Index> __findNearestPos(const Texture_t& vTexture, const Feature_t& vFeature) const;
	};

	//template class COrderIndependentTextureSynthesizer<int, 3>;
	//template class COrderIndependentTextureSynthesizer<float, 1>;
}
