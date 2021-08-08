#include "pch.h"
#include "OrderIndependentTextureSynthesizer.h"
#include "MipmapGenerator.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION:
template <typename Scalar_t, unsigned Channel>
Scalar_t COrderIndependentTextureSynthesizer<Scalar_t, Channel>::__computeDistance(const Feature_t& vLhs, const Feature_t& vRhs)
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
template <typename Scalar_t, unsigned Channel>
typename COrderIndependentTextureSynthesizer<Scalar_t, Channel>::NeighborOffset_t COrderIndependentTextureSynthesizer<Scalar_t, Channel>::__buildNeighborOffset(int vKernelSize)
{
	const int KernelOffset = vKernelSize / 2;
	const size_t KernelWidth = static_cast<size_t>(KernelOffset) * 2 + 1;
	NeighborOffset_t NeighborOffset;
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
template <typename Scalar_t, unsigned Channel>
void COrderIndependentTextureSynthesizer<Scalar_t, Channel>::execute(const Texture_t& vInput, const Eigen::MatrixXi& vMask, Texture_t& vioScene)
{
	m_NeighborOffset = __buildNeighborOffset(m_KernelSize);
	__buildPyramid(vInput, vMask, vioScene);
	SPyramidLayer LastLayer = m_Pyramid.front();
	for (size_t Layer = 1; Layer < m_Pyramid.size(); ++Layer)
	{
		auto& CurrentLayer = m_Pyramid[Layer];
		for (Eigen::Index RowId = 0; RowId < CurrentLayer._Mask.rows(); ++RowId)
			for (Eigen::Index ColId = 0; ColId < CurrentLayer._Mask.cols(); ++ColId)
				if (CurrentLayer._Mask.coeff(RowId, ColId) != 0)
				{
					auto Feature = __generateFeatureAt(LastLayer._Output, RowId, ColId);
					Eigen::Index NearestRowId, NearestColId;
					{
						Scalar_t MinDistance = std::numeric_limits<Scalar_t>::max();
						for (Eigen::Index i = 0; i < CurrentLayer._Input.rows(); ++i)
							for (Eigen::Index k = 0; k < CurrentLayer._Input.cols(); ++k)
							{
								auto Distance = __computeDistance(Feature, __generateFeatureAt(LastLayer._Input, i, k));

								if (MinDistance > Distance)
								{
									MinDistance = Distance;
									NearestRowId = i;
									NearestColId = k;
								}
							}
					}
					CurrentLayer._Output.coeffRef(RowId, ColId) = CurrentLayer._Input.coeff(NearestRowId, NearestColId);
				}
		LastLayer = CurrentLayer;
	}
	LastLayer = m_Pyramid.front();
	for (size_t Layer = 1; Layer < m_Pyramid.size(); ++Layer)
	{
		auto& CurrentLayer = m_Pyramid[Layer];
		for (Eigen::Index RowId = 0; RowId < CurrentLayer._Mask.rows(); ++RowId)
			for (Eigen::Index ColId = 0; ColId < CurrentLayer._Mask.cols(); ++ColId)
				if (CurrentLayer._Mask.coeff(RowId, ColId) != 0)
				{
					auto Feature = __generateFeatureAt(LastLayer._Output, RowId, ColId);
					auto TempFeature = __generateFeatureAt(CurrentLayer._Output, RowId, ColId);
					Eigen::Index NearestRowId, NearestColId;
					{
						Scalar_t MinDistance = std::numeric_limits<Scalar_t>::max();
						for (Eigen::Index i = 0; i < CurrentLayer._Input.rows(); ++i)
							for (Eigen::Index k = 0; k < CurrentLayer._Input.cols(); ++k)
							{
								auto Distance =  __computeDistance(Feature, __generateFeatureAt(LastLayer._Input, i, k)) / 4
									+ __computeDistance(TempFeature, __generateFeatureAt(CurrentLayer._Input, i, k));

								if (MinDistance > Distance)
								{
									MinDistance = Distance;
									NearestRowId = i;
									NearestColId = k;
								}
							}
					}
					CurrentLayer._Output.coeffRef(RowId, ColId) = CurrentLayer._Input.coeff(NearestRowId, NearestColId);
				}
		LastLayer = CurrentLayer;
	}
	vioScene = std::move(m_Pyramid.back()._Output);
}

//*****************************************************************
//FUNCTION: 
template <typename Scalar_t, unsigned Channel>
void COrderIndependentTextureSynthesizer<Scalar_t, Channel>::__buildPyramid(const Texture_t& vInput, const Eigen::MatrixXi& vMask, const Texture_t& vOutput)
{
	CMipmapGenerator<Eigen::Vector3i> TextureMipmapGenerator;
	TextureMipmapGenerator.setKernalSize(m_GaussianSize);
	CMipmapGenerator<int> MaskMipmapGenerator;
	MaskMipmapGenerator.setKernalSize(m_GaussianSize);
	auto InputPyramid = TextureMipmapGenerator.getGaussianStack(vInput, m_PyramidLayerNum);
	auto MaskPyramid = MaskMipmapGenerator.getGaussianStack(vMask, m_PyramidLayerNum);
	auto OutputPyramid = TextureMipmapGenerator.getGaussianStack(vOutput, m_PyramidLayerNum);

	m_Pyramid.clear();
	m_Pyramid.resize(m_PyramidLayerNum);
	for (auto& [Input ,Mask, Output] : m_Pyramid)
	{
		Input.swap(InputPyramid.back());
		InputPyramid.pop_back();
		Mask.swap(MaskPyramid.back());
		MaskPyramid.pop_back();
		Output.swap(OutputPyramid.back());
		OutputPyramid.pop_back();
	}
	std::ranges::reverse(m_Pyramid);
}

//*****************************************************************
//FUNCTION: 
template <typename Scalar_t, unsigned Channel>
typename COrderIndependentTextureSynthesizer<Scalar_t, Channel>::Feature_t COrderIndependentTextureSynthesizer<Scalar_t, Channel>::__generateFeatureAt(const Texture_t& vTexture, size_t vRowId, size_t vColId) const
{
	auto wrap = [](Eigen::Index vIndex, Eigen::Index vSize)
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
template <typename Scalar_t, unsigned Channel>
std::pair<Eigen::Index, Eigen::Index> COrderIndependentTextureSynthesizer<Scalar_t, Channel>::__findNearestPos(const Texture_t& vTexture, const Feature_t& vFeature) const
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
