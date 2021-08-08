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
		using NeighborOffset_t = std::vector<std::pair<int, int>>;
		struct SPyramidLayer
		{
			Texture_t _Input;
			Eigen::MatrixXi _Mask;
			Texture_t _Output;
		};
		//TODO: magic number
		int m_KernelSize = 9;
		NeighborOffset_t m_NeighborOffset;
		int m_PyramidLayerNum = 9;
		int m_GaussianSize = 9;
		std::vector<SPyramidLayer> m_Pyramid;
		
		static Scalar_t __computeDistance(const Feature_t& vLhs, const Feature_t& vRhs);
		static NeighborOffset_t __buildNeighborOffset(int vKernelSize);
		void __buildPyramid(const Texture_t& vInput, const Eigen::MatrixXi& vMask, const Texture_t& vOutput);
		Feature_t __generateFeatureAt(const Texture_t& vTexture, size_t vRowId, size_t vColId) const;
		std::pair<Eigen::Index, Eigen::Index> __findNearestPos(const Texture_t& vTexture, const Feature_t& vFeature) const;
	};

	//template class COrderIndependentTextureSynthesizer<int, 3>;
	//template class COrderIndependentTextureSynthesizer<float, 1>;
}
