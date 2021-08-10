#include "pch.h"
#include "OrderIndependentTextureSynthesizer.h"
#include "MipmapGenerator.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
//template <typename Scalar_t, unsigned Channel>
auto COrderIndependentTextureSynthesizer/*<Scalar_t, Channel>*/::__buildNeighborOffset(int vKernelSize) -> NeighborOffset_t
{
	const int KernelOffset = vKernelSize / 2;
	const int KernelWidth = KernelOffset * 2 + 1;
	NeighborOffset_t NeighborOffset;
	NeighborOffset.reserve(KernelWidth * KernelWidth);
	
	for (int i = -KernelOffset; i <= KernelOffset; ++i)
		for (int k = -KernelOffset; k <= KernelOffset; ++k)
			//if (i < 0 || (i == 0 && k < 0))
				NeighborOffset.emplace_back(i, k);
	
	NeighborOffset.shrink_to_fit();
	return NeighborOffset;
}

//*****************************************************************
//FUNCTION: 
//template <typename Scalar_t, unsigned Channel>
void COrderIndependentTextureSynthesizer/*<Scalar_t, Channel>*/::execute(const Texture_t& vInput, const Eigen::MatrixXi& vMask, Texture_t& vioScene)
{
	__initCache(vMask, vioScene);
	__initInputPyramid(vInput);
	__initTexture(m_InputPyramid.front(), m_Cache.front().front());
	m_NeighborOffset = __buildNeighborOffset(m_KernelSize);

	for (Eigen::Index RowId = 0; RowId < vioScene.rows(); ++RowId)
		for (Eigen::Index ColId = 0; ColId < vioScene.cols(); ++ColId)
			if (vMask.coeff(RowId, ColId) != 0)
				vioScene.coeffRef(RowId, ColId) = __synthesizePixel(m_PyramidLayer - 1, m_GenerationNum - 1, RowId, ColId);

	int Cnt = 0;
	for (const auto& i : m_Cache)
		for (const auto& k : i)
		{
			generateResultImage(k, "../TestData/Test019_Model/Cache/" + std::string("Cache") + std::to_string(Cnt++) + ".png");
		}
}

//*****************************************************************
//FUNCTION: 
//template <typename Scalar_t, unsigned Channel>
void COrderIndependentTextureSynthesizer/*<Scalar_t, Channel>*/::__initCache(const Eigen::MatrixXi& vMask, const Texture_t& vOutput)
{
	CMipmapGenerator<Eigen::Vector3i> TextureMipmapGenerator;
	TextureMipmapGenerator.setKernalSize(m_GaussianSize);
	auto OutputPyramid = TextureMipmapGenerator.getGaussianPyramid(vOutput, m_PyramidLayer, vMask);
	
	m_Cache.clear();
	m_Cache.reserve(m_PyramidLayer);
	for (const auto& Gaussian : OutputPyramid)
		m_Cache.emplace_back(m_GenerationNum, Gaussian);

	m_Cache.front().resize(1);
}

//*****************************************************************
//FUNCTION: 
//template <typename Scalar_t, unsigned Channel>
void COrderIndependentTextureSynthesizer/*<Scalar_t, Channel>*/::__initInputPyramid(const Texture_t& vTexture)
{
	CMipmapGenerator<Eigen::Vector3i> TextureMipmapGenerator;
	TextureMipmapGenerator.setKernalSize(m_GaussianSize);
	m_InputPyramid = TextureMipmapGenerator.getGaussianPyramid(vTexture, m_PyramidLayer);
}

//*****************************************************************
//FUNCTION: 
//template <typename Scalar_t, unsigned Channel>
void COrderIndependentTextureSynthesizer/*<Scalar_t, Channel>*/::__initTexture(const Texture_t& vFrom, Texture_t& voTo) const
{
	for (Eigen::Index RowId = 0; RowId < voTo.rows(); ++RowId)
		for (Eigen::Index ColId = 0; ColId < voTo.cols(); ++ColId)
		{
			auto& Item = voTo.coeffRef(RowId, ColId);
			if (!__isAvailable(Item))
			{
				Item = vFrom.coeff(
					hiveMath::hiveGenerateRandomInteger<Eigen::Index>(0, vFrom.rows() - 1),
					hiveMath::hiveGenerateRandomInteger<Eigen::Index>(0, vFrom.cols() - 1)
				);
			}
		}
}

//*****************************************************************
//FUNCTION: 
//template <typename Scalar_t, unsigned Channel>
void COrderIndependentTextureSynthesizer/*<Scalar_t, Channel>*/::__decrease(int& vioLayer, int& vioGeneration, Eigen::Index& vioRowId, Eigen::Index& vioColId) const
{
	if (vioGeneration > 0)
		--vioGeneration;
	else
	{
		--vioLayer;
		if (vioLayer == 0)
			vioGeneration = 0;
		else
			vioGeneration = m_GenerationNum - 1;
		vioRowId /= 2;
		vioColId /= 2;
	}
}

//*****************************************************************
//FUNCTION: 
//template <typename Scalar_t, unsigned Channel>
auto COrderIndependentTextureSynthesizer/*<Scalar_t, Channel>*/::__getInputAt(int vLayer, Eigen::Index vRowId, Eigen::Index vColId) const -> Texture_t::value_type
{
	vLayer = std::clamp(vLayer, 0, m_PyramidLayer - 1);
	const auto& Texture = m_InputPyramid[vLayer];
	
	__wrap(Texture.rows(), vRowId);
	__wrap(Texture.cols(), vColId);
	return Texture.coeff(vRowId, vColId);
}

//*****************************************************************
//FUNCTION: 
//template <typename Scalar_t, unsigned Channel>
auto COrderIndependentTextureSynthesizer/*<Scalar_t, Channel>*/::__getCacheAt(int vLayer, int vGeneration, Eigen::Index vRowId, Eigen::Index vColId) const -> Texture_t::value_type
{
	vLayer = std::clamp(vLayer, 0, m_PyramidLayer - 1);
	if (vLayer == 0)
		vGeneration = 0;
	else
		vGeneration = std::clamp(vGeneration, 0, m_GenerationNum - 1);
	const auto& Texture = m_Cache[vLayer][vGeneration];
	
	__wrap(Texture.rows(), vRowId);
	__wrap(Texture.cols(), vColId);
	return Texture.coeff(vRowId, vColId);
}

//*****************************************************************
//FUNCTION: 
//template <typename Scalar_t, unsigned Channel>
void COrderIndependentTextureSynthesizer/*<Scalar_t, Channel>*/::__addCacheEntry(int vLayer, int vGeneration, Eigen::Index vRowId, Eigen::Index vColId, const Texture_t::value_type& vValue)
{
	m_Cache[vLayer][vGeneration].coeffRef(vRowId, vColId) = vValue;
}

//*****************************************************************
//FUNCTION: 
//template <typename Scalar_t, unsigned Channel>
auto COrderIndependentTextureSynthesizer/*<Scalar_t, Channel>*/::__synthesizePixel(int vLayer, int vGeneration, Eigen::Index vRowId, Eigen::Index vColId) -> Texture_t::value_type
{
	auto CacheValue = __getCacheAt(vLayer, vGeneration, vRowId, vColId);
	if (!__isAvailable(CacheValue))
	{
		CacheValue = __findNearestValue(vLayer, vGeneration, __buildOutputFeatureAt(vLayer, vGeneration, vRowId, vColId));
		__addCacheEntry(vLayer, vGeneration, vRowId, vColId, CacheValue);
	}
	return CacheValue;
}

//*****************************************************************
//FUNCTION: 
//template <typename Scalar_t, unsigned Channel>
auto COrderIndependentTextureSynthesizer/*<Scalar_t, Channel>*/::__buildOutputFeatureAt(int vLayer, int vGeneration, Eigen::Index vRowId, Eigen::Index vColId) -> Feature_t
{
	__decrease(vLayer, vGeneration, vRowId, vColId);
	
	Eigen::Matrix<Scalar_t, Eigen::Dynamic, Channel> Feature(m_NeighborOffset.size(), Channel);
	for (size_t i = 0; i < m_NeighborOffset.size(); i++)
	{
		auto RowIdWithOffset = m_NeighborOffset[i].first + vRowId;
		auto ColIdWithOffset = m_NeighborOffset[i].second + vColId;

		auto CacheValue = __getCacheAt(vLayer, vGeneration, RowIdWithOffset, ColIdWithOffset);
		if (!__isAvailable(CacheValue))
			CacheValue = __synthesizePixel(vLayer, vGeneration, RowIdWithOffset, ColIdWithOffset);
		Feature.row(i) = CacheValue;
	}

	return Eigen::Map<Feature_t>(Feature.data(), Feature.size());
}

//*****************************************************************
//FUNCTION: 
//template <typename Scalar_t, unsigned Channel>
auto COrderIndependentTextureSynthesizer/*<Scalar_t, Channel>*/::__buildInputFeatureAt(int vLayer, int vGeneration, Eigen::Index vRowId, Eigen::Index vColId) const -> Feature_t
{
	__decrease(vLayer, vGeneration, vRowId, vColId);
	
	Eigen::Matrix<Scalar_t, Eigen::Dynamic, Channel> Feature(m_NeighborOffset.size(), Channel);
	for (size_t i = 0; i < m_NeighborOffset.size(); i++)
	{
		auto RowIdWithOffset = m_NeighborOffset[i].first + vRowId;
		auto ColIdWithOffset = m_NeighborOffset[i].second + vColId;

		Feature.row(i) = __getInputAt(vLayer, RowIdWithOffset, ColIdWithOffset);
	}

	return Eigen::Map<Feature_t>(Feature.data(), Feature.size());
}

//*****************************************************************
//FUNCTION: 
//template <typename Scalar_t, unsigned Channel>
auto COrderIndependentTextureSynthesizer/*<Scalar_t, Channel>*/::__findNearestValue(int vLayer, int vGeneration, const Feature_t& vFeature) const -> Texture_t::value_type
{
	const auto& Input = m_InputPyramid[vLayer];
	Texture_t::value_type NearestValue;
	auto MinDistance = std::numeric_limits<Scalar_t>::max();
	
	for (Eigen::Index RowId = 0; RowId < Input.rows(); ++RowId)
		for (Eigen::Index ColId = 0; ColId < Input.cols(); ++ColId)
		{
			auto Distance = __computeDistance(vFeature, __buildInputFeatureAt(vLayer, vGeneration, RowId, ColId));
			if (MinDistance > Distance)
			{
				MinDistance = Distance;
				NearestValue = Input.coeff(RowId, ColId);
			}
		}
	return NearestValue;
}
