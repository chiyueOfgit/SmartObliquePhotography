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
				//生成邻居掩码
				//根据邻居掩码生成Feature
				const int KernelOffset = m_KernelSize / 2;
				const int KernelWidth = KernelOffset * 2 + 1;
				NeighborMask_t NeighborMask(KernelWidth, KernelWidth);
				std::vector<Texture_t::value_type> Feature;
				for (int i = -KernelOffset; i <= KernelOffset; ++i)
					for (int k = -KernelOffset; k <= KernelOffset; ++k)
					{
						auto RowIdWithOffset = (i + RowId + vMask.rows()) % vMask.rows();
						auto ColIdWithOffset = (k + ColId + vMask.cols()) % vMask.cols();

						//TODO: 满足前者但不满足后者也不行
						if (i < 0 || (i == 0 && k < 0) || vMask.coeff(RowIdWithOffset, ColIdWithOffset) == 0)
						{
							NeighborMask(i + KernelOffset, k + KernelOffset) = 1;
							Feature.emplace_back(vioScene.coeff(RowIdWithOffset, ColIdWithOffset));
						}
						else
							NeighborMask(i + KernelOffset, k + KernelOffset) = 0;
					}

				//根据邻居掩码和Feature __findNearestPos
				auto [NearestRowId, NearestColId] = __findNearestPos(vInput, NeighborMask, __flatFeature(Feature));
				vioScene.coeffRef(RowId, ColId) = vInput.coeff(NearestRowId, NearestColId);
			}
}

//*****************************************************************
//FUNCTION: 
template <typename Scalar_t, unsigned Channel>
typename CTextureSynthesizer<Scalar_t, Channel>::Feature_t CTextureSynthesizer<Scalar_t, Channel>::__flatFeature(const std::vector<typename Texture_t::value_type>& vFeature) const
{
	Feature_t FlatFeature(vFeature.size() * Channel);
	for (size_t i = 0; i < vFeature.size(); ++i)
		for (unsigned k = 0; k < Channel; ++k)
			FlatFeature((i * Channel) + k) = vFeature[i][k];

	return FlatFeature;
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
			//根据他人的邻居掩码生成Feature
			const int KernelOffset = m_KernelSize / 2;
			std::vector<Texture_t::value_type> Feature;
			for (int i = -KernelOffset; i <= KernelOffset; ++i)
				for (int k = -KernelOffset; k <= KernelOffset; ++k)
				{
					auto RowIdWithOffset = (i + RowId + vTexture.rows()) % vTexture.rows();
					auto ColIdWithOffset = (k + ColId + vTexture.cols()) % vTexture.cols();

					if (vNeighborMask(i + KernelOffset, k + KernelOffset) != 0)
						Feature.emplace_back(vTexture.coeff(RowIdWithOffset, ColIdWithOffset));
				}

			//范数的值类型为Scalar_t，精度足够
			auto Distance = (__flatFeature(Feature) - vFeature).squaredNorm();
			if (MinDistance > Distance)
			{
				MinDistance = Distance;
				MinPos = { RowId, ColId };
			}
		}
	return MinPos;
}
