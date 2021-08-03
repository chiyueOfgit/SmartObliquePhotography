#pragma once
namespace hiveObliquePhotography::PointCloudRetouch
{

	template <typename Scalar_t, unsigned Channel>
	class CTextureSynthesizer
	{
	public:
		using Texture_t = Eigen::Matrix<Eigen::Matrix<Scalar_t, Channel, 1>, Eigen::Dynamic, Eigen::Dynamic>;
		CTextureSynthesizer() = default;
		~CTextureSynthesizer() = default;

		void execute(const Texture_t& vInput, const Eigen::MatrixXi& vMask, Texture_t& vioScene);

	private:
		using Feature_t = std::vector<Eigen::Matrix<Scalar_t, Channel, 1>>;
		using NeighborMask_t = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>;
		//TODO: magic number
		int m_KernelSize = 9;

		Feature_t __generateFeatureAt(const Texture_t& vTexture, const NeighborMask_t& vNeighborMask, size_t vRowId, size_t vColId) const;
		Scalar_t __computeDistance(const Feature_t& vLhs, const Feature_t& vRhs) const;
		std::pair<Eigen::Index, Eigen::Index> __findNearestPos(const Texture_t& vTexture, const NeighborMask_t& vNeighborMask, const Feature_t& vFeature) const;
	};

	template class CTextureSynthesizer<int, 3>;
	template class CTextureSynthesizer<float, 1>;
}
