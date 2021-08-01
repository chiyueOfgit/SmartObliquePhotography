#include "pch.h"
#include "TextureSynthesizer.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
template <typename Color_t>
bool CTextureSynthesizer<Color_t>::init(const hiveConfig::CHiveConfig* vConfig)
{
	_ASSERTE(vConfig);
	m_pConfig = vConfig;

	return true;
}

//*****************************************************************
//FUNCTION: 
template <typename Color_t>
void CTextureSynthesizer<Color_t>::execute(const Texture_t& vInput, const Eigen::MatrixXi& vMask, Texture_t& vioScene)
{
	m_NeighborOffset = __generateNeighborOffset(m_KernelSize);
	for (Eigen::Index i = 0; i < vMask.rows(); ++i) 
		for (Eigen::Index k = 0; k < vMask.cols(); ++k)
			if (vMask.coeff(i, k) != 0)
			{
				auto Feature = __generateFeatureAt(vioScene, i, k);
				auto [NearestRowId, NearestColId] = __findNearestPos(vInput, Feature);
				vioScene.coeffRef(i, k) = vInput.coeff(NearestRowId, NearestColId);
			}
}

//*****************************************************************
//FUNCTION: 
template <typename Color_t>
std::vector<std::pair<int, int>> CTextureSynthesizer<Color_t>::__generateNeighborOffset(int vKernelSize) const
{
	const auto KernelOffset = m_KernelSize / 2;
	std::vector<std::pair<int, int>> NeighborOffset;
	for (auto i = -KernelOffset; i <= KernelOffset; ++i)
		for (auto k = -KernelOffset; k <= KernelOffset; ++k)
		{
			if (i < 0 || (i == 0 && k < 0))
				NeighborOffset.emplace_back(i, k);
		}
	return NeighborOffset;
}

//*****************************************************************
//FUNCTION: 
template <typename Color_t>
typename CTextureSynthesizer<Color_t>::Feature_t CTextureSynthesizer<Color_t>::__generateFeatureAt(const Texture_t& vTexture, size_t vRowId, size_t vColId) const
{
	Feature_t Feature(m_NeighborOffset.size());
	for (size_t i = 0; i < m_NeighborOffset.size(); i++)
	{
		auto RowId = (vRowId + m_NeighborOffset[i].first + vTexture.rows()) % vTexture.rows();
		auto ColId = (vColId + m_NeighborOffset[i].second + vTexture.cols()) % vTexture.cols();
		Feature[i] = vTexture.coeff(RowId, ColId);
	}
	
	return Feature;
}

//*****************************************************************
//FUNCTION: 
template <typename Color_t>
std::pair<Eigen::Index, Eigen::Index> CTextureSynthesizer<Color_t>::__findNearestPos(const Texture_t& vTexture, const Feature_t& vFeature) const
{
	float MinDistance = FLT_MAX;
	std::pair<Eigen::Index, Eigen::Index> MinPos;
	for (Eigen::Index i = 0; i < vTexture.rows(); ++i)
		for (Eigen::Index k = 0; k < vTexture.cols(); ++k)
		{
			auto Feature = __generateFeatureAt(vTexture, i, k);
			auto Distance = __computeFeatureDistance(Feature, vFeature);
			if (MinDistance > Distance)
			{
				MinDistance = Distance;
				MinPos = {i, k};
			}
		}
	return MinPos;
}

//*****************************************************************
//FUNCTION: 
template <typename Color_t>
float CTextureSynthesizer<Color_t>::__computeFeatureDistance(const Feature_t& vLhs, const Feature_t& vRhs) const
{
	_ASSERTE(vLhs.size() == vRhs.size());
	float Sum = 0.0f;
	for (Eigen::Index  i = 0; i < vLhs.size(); i++)
		Sum += (vLhs[i] - vRhs[i]).cast<float>().squaredNorm();
	return Sum;
}
