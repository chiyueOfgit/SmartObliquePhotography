#include "pch.h"
#include "MultithreadTextureSynthesizer.h"
#include "MipmapGenerator.h"

#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
template <typename Scalar_t, unsigned Channel>
std::vector<std::pair<int, int>> CMultithreadTextureSynthesizer<Scalar_t, Channel>::__buildNeighborOffset(int vKernelSize)
{
	const int KernelOffset = vKernelSize / 2;
	const int KernelWidth = KernelOffset * 2 + 1;
	std::vector<std::pair<int, int>> NeighborOffset;
	NeighborOffset.reserve(KernelWidth * KernelWidth);
	
	for (int i = -KernelOffset; i <= KernelOffset; ++i)
		for (int k = -KernelOffset; k <= KernelOffset; ++k)
			if (std::hypot(i ,k) <= KernelOffset)
				NeighborOffset.emplace_back(i, k);
	
	NeighborOffset.shrink_to_fit();
	return NeighborOffset;
}

//*****************************************************************
//FUNCTION: 
template <typename Scalar_t, unsigned Channel>
void CMultithreadTextureSynthesizer<Scalar_t, Channel>::execute(const Texture_t& vInput, const Eigen::MatrixXi& vMask, Texture_t& vioScene)
{
	__initCache(vMask, vioScene);
	__initInputPyramid(vInput);
	__initTexture(m_InputPyramid.front(), m_Cache.front().front());
	m_NeighborOffset = __buildNeighborOffset(m_KernelSize);

	int Layer = 0, Generation = 0;
	while (__increase(Layer, Generation))
		__synthesizeTexture(Layer, Generation);
	
	int Cnt = 0;
	for (const auto& i : m_Cache)
		for (const auto& k : i)
			__generateResultImage(k, "../TestData/Test019_Model/Cache/" + std::string("Cache") + std::to_string(Cnt++) + ".png");

	vioScene = std::move(m_Cache.back().back());
}

//*****************************************************************
//FUNCTION: 
template <typename Scalar_t, unsigned Channel>
void CMultithreadTextureSynthesizer<Scalar_t, Channel>::__initCache(const Eigen::MatrixXi& vMask, const Texture_t& vOutput)
{
	CMipmapGenerator<Color_t> TextureMipmapGenerator;
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
template <typename Scalar_t, unsigned Channel>
void CMultithreadTextureSynthesizer<Scalar_t, Channel>::__initInputPyramid(const Texture_t& vTexture)
{
	CMipmapGenerator<Color_t> TextureMipmapGenerator;
	TextureMipmapGenerator.setKernalSize(m_GaussianSize);
	m_InputPyramid = TextureMipmapGenerator.getGaussianPyramid(vTexture, m_PyramidLayer);
}

//*****************************************************************
//FUNCTION: 
template <typename Scalar_t, unsigned Channel>
void CMultithreadTextureSynthesizer<Scalar_t, Channel>::__initTexture(const Texture_t& vFrom, Texture_t& voTo) const
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
//
////*****************************************************************
////FUNCTION: 
template <typename Scalar_t, unsigned Channel>
bool CMultithreadTextureSynthesizer<Scalar_t, Channel>::__increase(int& vioLayer, int& vioGeneration) const
{
	if (vioGeneration < m_GenerationNum - 1 && vioLayer > 0)
		++vioGeneration;
	else if (vioLayer < m_PyramidLayer - 1)
	{
		++vioLayer;
		vioGeneration = 0;
	}
	else
		return false;
	return true;
}

////*****************************************************************
////FUNCTION: 
template <typename Scalar_t, unsigned Channel>
void CMultithreadTextureSynthesizer<Scalar_t, Channel>::__decrease(int& vioLayer, int& vioGeneration, Eigen::Index& vioRowId, Eigen::Index& vioColId) const
{
	if (vioGeneration > 0)
		--vioGeneration;
	else if (vioLayer > 0)
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
template <typename Scalar_t, unsigned Channel>
void CMultithreadTextureSynthesizer<Scalar_t, Channel>::__synthesizeTexture(int vLayer, int vGeneration)
{
	auto& Texture = m_Cache[vLayer][vGeneration];
	const auto Height = Texture.rows();
	const auto Width = Texture.cols();

#pragma omp parallel for
	for (int i = 0; i < Height * Width; ++i)
	{
		auto& Item = Texture.coeffRef(i);
		if (!__isAvailable(Item))
			Item = __findNearestValue(vLayer, vGeneration, __buildOutputFeatureAt(vLayer, vGeneration, i % Height, i / Height));
	}
}

//*****************************************************************
//FUNCTION: 
template <typename Scalar_t, unsigned Channel>
auto CMultithreadTextureSynthesizer<Scalar_t, Channel>::__buildOutputFeatureAt(int vLayer, int vGeneration, Eigen::Index vRowId, Eigen::Index vColId) const -> Feature_t
{
	__decrease(vLayer, vGeneration, vRowId, vColId);
	return __buildFeatureAt(m_Cache[vLayer][vGeneration], vRowId, vColId);
}

//*****************************************************************
//FUNCTION: 
template <typename Scalar_t, unsigned Channel>
auto CMultithreadTextureSynthesizer<Scalar_t, Channel>::__buildInputFeatureAt(int vLayer, int vGeneration, Eigen::Index vRowId, Eigen::Index vColId) const -> Feature_t
{
	__decrease(vLayer, vGeneration, vRowId, vColId);
	return __buildFeatureAt(m_InputPyramid[vLayer], vRowId, vColId);
}

//*****************************************************************
//FUNCTION: 
template <typename Scalar_t, unsigned Channel>
auto CMultithreadTextureSynthesizer<Scalar_t, Channel>::__buildFeatureAt(const Texture_t& vTexture, Eigen::Index vRowId, Eigen::Index vColId) const->Feature_t
{
	Eigen::Matrix<Scalar_t, Channel, Eigen::Dynamic> Feature(Channel, m_NeighborOffset.size());
	for (Eigen::Index It = 0; auto [RowOffset, ColOffset] : m_NeighborOffset)
	{
		auto RowIdWithOffset = __wrap(vTexture.rows(), vRowId + RowOffset);
		auto ColIdWithOffset = __wrap(vTexture.cols(), vColId + ColOffset);
		Feature.col(It++) = vTexture.coeff(RowIdWithOffset, ColIdWithOffset);
	}	
	return Feature;
}

//*****************************************************************
//FUNCTION: 
template <typename Scalar_t, unsigned Channel>
auto CMultithreadTextureSynthesizer<Scalar_t, Channel>::__findNearestValue(int vLayer, int vGeneration, const Feature_t& vFeature) const -> Color_t
{
	const auto& Texture = m_InputPyramid[vLayer];
	Color_t NearestValue;
	auto MinDistance = std::numeric_limits<Scalar_t>::max();
	
	for (Eigen::Index RowId = 0; RowId < Texture.rows(); ++RowId)
		for (Eigen::Index ColId = 0; ColId < Texture.cols(); ++ColId)
		{
			auto Distance = __computeDistance(vFeature, __buildInputFeatureAt(vLayer, vGeneration, RowId, ColId));
			if (MinDistance > Distance)
			{
				MinDistance = Distance;
				NearestValue = Texture.coeff(RowId, ColId);
			}
		}
	return NearestValue;
}

//*****************************************************************
//FUNCTION: 
template <typename Scalar_t, unsigned Channel>
void CMultithreadTextureSynthesizer<Scalar_t, Channel>::__generateResultImage(const Texture_t& vTexture, const std::string& vOutputImagePath) const
{
	const auto Width = vTexture.cols();
	const auto Height = vTexture.rows();
	const auto BytesPerPixel = 3;
	auto ResultImage = new unsigned char[Width * Height * BytesPerPixel];
	for (auto i = 0; i < Height; i++)
		for (auto k = 0; k < Width; k++)
		{
			const auto& Item = vTexture.coeff(i, k);
			auto Offset = (i * Width + k) * BytesPerPixel;
			if (__isAvailable(Item))
			{
				ResultImage[Offset] = Item[0];
				ResultImage[Offset + 1] = Item[1];
				ResultImage[Offset + 2] = Item[2];
			}
			else
			{
				ResultImage[Offset] = 0;
				ResultImage[Offset + 1] = 0;
				ResultImage[Offset + 2] = 0;
			}
		}
	stbi_write_png(vOutputImagePath.c_str(), Width, Height, BytesPerPixel, ResultImage, 0);
	stbi_image_free(ResultImage);
}
