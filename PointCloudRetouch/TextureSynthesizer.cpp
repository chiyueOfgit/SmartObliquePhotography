#include "pch.h"
#include "TextureSynthesizer.h"

#include <pcl/features/feature.h>

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
template <typename Scalar_t, unsigned Channel>
void CTextureSynthesizer<Scalar_t, Channel>::execute(const Texture_t& vInput, const Eigen::MatrixXi& vMask, Texture_t& vioScene)
{
	for (Eigen::Index RowId = 0; RowId < vMask.rows(); ++RowId)
		for (Eigen::Index ColId = 0; ColId < vMask.cols(); ++ColId)
			if (vMask.coeff(RowId, ColId) != 0)
			{
				const int KernelOffset = m_KernelSize / 2;
				const int KernelWidth = KernelOffset * 2 + 1;
				NeighborMask_t NeighborMask(KernelWidth, KernelWidth);
				NeighborMask.setConstant(0);
				for (int i = -KernelOffset; i <= KernelOffset; ++i)
					for (int k = -KernelOffset; k <= KernelOffset; ++k)
					{
						auto RowIdWithOffset = (i + RowId + vMask.rows()) % vMask.rows();
						auto ColIdWithOffset = (k + ColId + vMask.cols()) % vMask.cols();

						if (vMask.coeff(RowIdWithOffset, ColIdWithOffset) == 0)
							NeighborMask(i + KernelOffset, k + KernelOffset) = 1;
						else if (i < 0 || (i == 0 && k < 0))
						{
							if (i + RowId >= 0 && i + RowId < vMask.rows() && k + ColId >= 0 && k + ColId < vMask.cols())
							{
								NeighborMask(i + KernelOffset, k + KernelOffset) = 1;
							}
						}
					}

				auto Feature = __generateFeatureAt(vioScene, NeighborMask, RowId, ColId);
				auto [NearestRowId, NearestColId] = __findNearestPos(vInput, NeighborMask, Feature);
				vioScene.coeffRef(RowId, ColId) = vInput.coeff(NearestRowId, NearestColId);
			}
}

//*****************************************************************
//FUNCTION:
template <typename Scalar_t, unsigned Channel>
Scalar_t CTextureSynthesizer<Scalar_t, Channel>::__computeDistance(const Feature_t& vLhs, const Feature_t& vRhs) const
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
typename CTextureSynthesizer<Scalar_t, Channel>::Feature_t CTextureSynthesizer<Scalar_t, Channel>::__generateFeatureAt(const Texture_t& vTexture, const NeighborMask_t& vNeighborMask, size_t vRowId, size_t vColId) const
{
	const int KernelOffset = m_KernelSize / 2;
	Feature_t Feature;
	Feature.reserve((KernelOffset * 2 + 1) * (KernelOffset * 2 + 1));
	for (int i = -KernelOffset; i <= KernelOffset; ++i)
		for (int k = -KernelOffset; k <= KernelOffset; ++k)
			if (vNeighborMask(i + KernelOffset, k + KernelOffset) != 0)
			{
				auto RowIdWithOffset = (i + vRowId + vTexture.rows()) % vTexture.rows();
				auto ColIdWithOffset = (k + vColId + vTexture.cols()) % vTexture.cols();
				Feature.emplace_back(vTexture.coeff(RowIdWithOffset, ColIdWithOffset));
			}

	return Feature;
}

//*****************************************************************
//FUNCTION: 
template <typename Scalar_t, unsigned Channel>
std::pair<Eigen::Index, Eigen::Index> CTextureSynthesizer<Scalar_t, Channel>::__findNearestPos(const Texture_t& vTexture, const NeighborMask_t& vNeighborMask, const Feature_t& vFeature) const
{
	float MinDistance = FLT_MAX;
	std::pair<Eigen::Index, Eigen::Index> MinPos;
	for (Eigen::Index RowId = 0; RowId < vTexture.rows(); ++RowId)
		for (Eigen::Index ColId = 0; ColId < vTexture.cols(); ++ColId)
		{
			auto Distance = __computeDistance(vFeature, __generateFeatureAt(vTexture, vNeighborMask, RowId, ColId));
			
			if (MinDistance > Distance)
			{
				MinDistance = Distance;
				MinPos = { RowId, ColId };
			}
		}
	return MinPos;
}
