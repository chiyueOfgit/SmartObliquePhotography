#pragma once

namespace hiveObliquePhotography::PointCloudRetouch
{
	template <typename Color_t>
	class CTextureSynthesizer
	{
	public:
		using Texture_t = Eigen::Matrix<Color_t, -1, -1>;
		using Feature_t = Eigen::Matrix<Color_t, -1, 1>;
		
		CTextureSynthesizer() = default;
		~CTextureSynthesizer() = default;

		bool init(const hiveConfig::CHiveConfig* vConfig);
		void execute(const Texture_t& vInput, const Eigen::MatrixXi& vMask, Texture_t& vioScene);

	private:
		int m_KernelSize = 7;
		std::vector<std::pair<int, int>> m_NeighborOffset;

		const hiveConfig::CHiveConfig* m_pConfig = nullptr;
		
		std::vector<std::pair<int, int>> __generateNeighborOffset(int vKernelSize) const;
		Feature_t __generateFeatureAt(const Texture_t& vTexture, size_t vRowId, size_t vColId) const;
		std::pair<Eigen::Index, Eigen::Index> __findNearestPos(const Texture_t& vTexture, const Feature_t& vFeature) const;
		float __computeFeatureDistance(const Feature_t& vLhs, const Feature_t& vRhs) const;
	};

	template class CTextureSynthesizer<Eigen::Vector3i>;
	template class CTextureSynthesizer<Eigen::Matrix<float, 1, 1>>;
}
