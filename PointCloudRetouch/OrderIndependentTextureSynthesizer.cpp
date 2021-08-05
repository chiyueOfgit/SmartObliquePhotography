#include "pch.h"
#include "OrderIndependentTextureSynthesizer.h"
#include "Mipmap.hpp"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION:
//template <typename Scalar_t, unsigned Channel>
Scalar_t COrderIndependentTextureSynthesizer/*<Scalar_t, Channel>*/::__computeDistance(const Feature_t& vLhs, const Feature_t& vRhs)
{
	_ASSERTE(vLhs.size() == vRhs.size());
	const auto Size = vLhs.size();

	Eigen::Matrix<Scalar_t, Eigen::Dynamic, 1> FlatFeature(Size * Channel);
	for (size_t i = 0; i < Size; ++i)
		for (unsigned k = 0; k < Channel; ++k)
			FlatFeature((i * Channel) + k) = vLhs[i][k] - vRhs[i][k];

	return FlatFeature.squaredNorm();
}

//*****************************************************************
//FUNCTION: 
//template <typename Scalar_t, unsigned Channel>
std::vector<std::pair<int, int>> COrderIndependentTextureSynthesizer/*<Scalar_t, Channel>*/::__buildNeighborOffset(int vKernelSize)
{
	const int KernelOffset = vKernelSize / 2;
	const size_t KernelWidth = static_cast<size_t>(KernelOffset) * 2 + 1;
	std::vector<std::pair<int, int>> NeighborOffset;
	NeighborOffset.reserve(KernelWidth * KernelWidth);
	
	for (int i = -KernelOffset; i <= KernelOffset; ++i)
		for (int k = -KernelOffset; k <= KernelOffset; ++k)
		{
			//if (i < 0 || (i == 0 && k < 0))
				NeighborOffset.emplace_back(i, k);
		}
	
	NeighborOffset.shrink_to_fit();
	return NeighborOffset;
}

//*****************************************************************
//FUNCTION: 
//template <typename Scalar_t, unsigned Channel>
void COrderIndependentTextureSynthesizer/*<Scalar_t, Channel>*/::execute(const Texture_t& vInput, const Eigen::MatrixXi& vMask, Texture_t& vioScene)
{
	__buildPyramid(vInput, vMask, vioScene);
	//pass1: 自上而下遍历金字塔，注意需要上一层级的邻域
	//pass2: 自上而下遍历金字塔，注意还需要上一层级的邻域
	//输入金字塔最底端的结果
	
	m_NeighborOffset = __buildNeighborOffset(m_KernelSize);
	for (Eigen::Index RowId = 0; RowId < vMask.rows(); ++RowId)
		for (Eigen::Index ColId = 0; ColId < vMask.cols(); ++ColId)
			if (vMask.coeff(RowId, ColId) != 0)
			{
				auto Feature = __generateFeatureAt(vioScene, RowId, ColId);
				auto [NearestRowId, NearestColId] = __findNearestPos(vInput, Feature);
				vioScene.coeffRef(RowId, ColId) = vInput.coeff(NearestRowId, NearestColId);
			}
}

//*****************************************************************
//FUNCTION: 
//template <typename Scalar_t, unsigned Channel>
void COrderIndependentTextureSynthesizer/*<Scalar_t, Channel>*/::__buildPyramid(const Texture_t& vInput, const Eigen::MatrixXi& vMask, const Texture_t& vOutput)
{
	m_Pyramid.clear();
	m_Pyramid.emplace_back(vInput, vMask, vOutput);
	while (m_Pyramid.size() < m_PyramidLayer)
	{
		const auto& LastLayer = m_Pyramid.back();
		m_Pyramid.emplace_back(Utility::getMipMap(std::get<0>(LastLayer)), Utility::getMipMap(std::get<1>(LastLayer)), Utility::getMipMap(std::get<2>(LastLayer)));
	}
	std::ranges::reverse(m_Pyramid);
}

//*****************************************************************
//FUNCTION: 
//template <typename Scalar_t, unsigned Channel>
typename COrderIndependentTextureSynthesizer/*<Scalar_t, Channel>*/::Feature_t COrderIndependentTextureSynthesizer/*<Scalar_t, Channel>*/::__generateFeatureAt(const Texture_t& vTexture, size_t vRowId, size_t vColId) const
{
	auto wrap = [](auto vIndex, auto vSize)
	{
		if (vIndex < 0)
			vIndex += vSize;
		else if (vIndex >= vSize)
			vIndex -= vSize;
		return vIndex;
	};
	
	Feature_t Feature;
	Feature.reserve(m_NeighborOffset.size());
	for (auto& [i, k] : m_NeighborOffset)
	{
		auto RowIdWithOffset = wrap(i + vRowId, vTexture.rows());
		auto ColIdWithOffset = wrap(k + vColId, vTexture.cols());

		Feature.push_back(vTexture.coeff(RowIdWithOffset, ColIdWithOffset));
	}
	return Feature;
}

//*****************************************************************
//FUNCTION: 
//template <typename Scalar_t, unsigned Channel>
std::pair<Eigen::Index, Eigen::Index> COrderIndependentTextureSynthesizer/*<Scalar_t, Channel>*/::__findNearestPos(const Texture_t& vTexture, const Feature_t& vFeature) const
{
	Scalar_t MinDistance = std::numeric_limits<Scalar_t>::max();
	std::pair<Eigen::Index, Eigen::Index> MinPos;
	for (Eigen::Index RowId = 0; RowId < vTexture.rows(); ++RowId)
		for (Eigen::Index ColId = 0; ColId < vTexture.cols(); ++ColId)
		{
			auto Distance = __computeDistance(vFeature, __generateFeatureAt(vTexture, RowId, ColId));
			
			if (MinDistance > Distance)
			{
				MinDistance = Distance;
				MinPos = { RowId, ColId };
			}
		}
	return MinPos;
}
