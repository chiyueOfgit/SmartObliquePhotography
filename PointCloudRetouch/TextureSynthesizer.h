#pragma once
namespace hiveObliquePhotography::PointCloudRetouch
{
	template <typename Scalar_t, unsigned Channel>
	class CTextureSynthesizer
	{
	public:
		using Texture_t = Eigen::Matrix<Eigen::Matrix<Scalar_t, Channel, 1>, Eigen::Dynamic, Eigen::Dynamic>;
		//Feature_t是一维数组，后续实装kd树可用
		using Feature_t = Eigen::Matrix<Scalar_t, Eigen::Dynamic, 1>;
		using NeighborMask_t = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>;

		CTextureSynthesizer() = default;
		~CTextureSynthesizer() = default;

		void execute(const Texture_t& vInput, const Eigen::MatrixXi& vMask, Texture_t& vioScene);

	private:
		//TODO: magic number
		int m_KernelSize = 5;

		Feature_t __flatFeature(const std::vector<typename Texture_t::value_type>& vFeature) const;
		std::pair<Eigen::Index, Eigen::Index> __findNearestPos(const Texture_t& vTexture, const NeighborMask_t& vNeighborMask, const Feature_t& vFeature) const;
	};

	template class CTextureSynthesizer<int, 3>;
	template class CTextureSynthesizer<float, 1>;
}
