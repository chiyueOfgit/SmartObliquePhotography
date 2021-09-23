#include "pch.h"
#include "TreeBasedTextureSynthesizer.h"
#include "MipmapGenerator.h"
#include <tbb/parallel_for.h>

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
std::vector<std::pair<int, int>> CTreeBasedTextureSynthesizer<Scalar_t, Channel>::__buildNeighborOffset(int vKernelSize)
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
void CTreeBasedTextureSynthesizer<Scalar_t, Channel>::execute(const Texture_t& vInput, const Eigen::MatrixXi& vMask, Texture_t& vioScene)
{
	__initCache(vMask, vioScene);
	//__initTexture(vInput, m_Cache.front().front());
	__initTextureWithNeighborMask(vInput, m_Cache.front().front());
	m_NeighborOffset = __buildNeighborOffset(m_KernelSize);
	__initSearchSet(vInput, m_NeighborOffset.size() * Channel);

	for (int Layer = 1; Layer < m_PyramidLayer; ++Layer)
	{
		//upsample
		const auto& From = m_Cache[Layer - 1].back();
		auto& To = m_Cache[Layer].front();
		for (Eigen::Index RowId = 0; RowId < From.rows(); ++RowId)
			for (Eigen::Index ColId = 0; ColId < From.cols(); ++ColId)
			{
				constexpr std::pair<int, int> Offset[] = { { 0, 0 }, { 0, 1 }, { 1, 0 }, { 1, 1 } };
				for (auto [i, k] : Offset)
				{
					auto Row = __wrap(To.rows(), 2 * RowId + i);
					auto Col = __wrap(To.cols(), 2 * ColId + k);
					auto& Item = To(Row, Col);
					if (!__isAvailable(Item))
						Item = From(RowId, ColId);
				}
			}
		
		for (int Generation = 1; Generation < m_GenerationNum; ++Generation)
			__synthesizeTexture(Layer, Generation);
	}
	
	int Cnt = 0;
	for (const auto& i : m_Cache)
		for (const auto& k : i)
			__generateResultImage(k, "../TestData/Test019_Model/Cache/" + std::string("Cache") + std::to_string(Cnt++) + ".png");

	vioScene = std::move(m_Cache.back().back());
}

//*****************************************************************
//FUNCTION: 
template <typename Scalar_t, unsigned Channel>
void CTreeBasedTextureSynthesizer<Scalar_t, Channel>::__initCache(const Eigen::MatrixXi& vMask, const Texture_t& vOutput)
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
void CTreeBasedTextureSynthesizer<Scalar_t, Channel>::__initSearchSet(const Texture_t& vTexture, int vFeatureLength)
{
	CMipmapGenerator<Color_t> TextureMipmapGenerator;
	TextureMipmapGenerator.setKernalSize(m_GaussianSize);	
	auto InputPyramid = TextureMipmapGenerator.getGaussianPyramid(vTexture, m_PyramidLayer);
	
	m_SearchSet.reserve(m_PyramidLayer);
	for (auto Layer = 0; Layer < InputPyramid.size(); Layer++)
	{
		const auto& Input = InputPyramid[Layer];

		Eigen::Matrix<Scalar_t, -1, -1, Eigen::RowMajor> FeatureSet(Input.rows() * Input.cols(), vFeatureLength);
		Eigen::Matrix<Scalar_t, -1, Channel> ColorSet(Input.rows() * Input.cols(), Channel);
		int Row = 0;
		for (int i = 0; i < Input.rows(); i++)
			for (int k = 0; k < Input.cols(); k++)
			{
				FeatureSet.row(Row) = __buildFeatureAt(Input, i, k);
				ColorSet.row(Row) = Input(i, k);
				Row++;
			}
		
		flann::Matrix InputIndices(FeatureSet.data(), FeatureSet.rows(), FeatureSet.cols());
		auto pTree = new flann::Index<flann::L2<Scalar_t>>(InputIndices, flann::KDTreeIndexParams(4));
		pTree->buildIndex();
		m_SearchSet.emplace_back(pTree, std::move(FeatureSet), std::move(ColorSet));
	}
}

//*****************************************************************
//FUNCTION: 
template <typename Scalar_t, unsigned Channel>
void CTreeBasedTextureSynthesizer<Scalar_t, Channel>::__initTexture(const Texture_t& vFrom, Texture_t& voTo) const
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
template <typename Scalar_t, unsigned Channel>
void CTreeBasedTextureSynthesizer<Scalar_t, Channel>::__initTextureWithNeighborMask(const Texture_t& vFrom, Texture_t& voTo) const
{
	CMipmapGenerator<Color_t> TextureMipmapGenerator;
	TextureMipmapGenerator.setKernalSize(m_GaussianSize);
	auto InputPyramid = TextureMipmapGenerator.getGaussianPyramid(vFrom, m_PyramidLayer);
	int Suitable = 0;
	/*for (int i = 0; i < InputPyramid.size(); i++)
	{
		if (abs(InputPyramid[i].rows() - voTo.rows()) < abs(InputPyramid[Suitable].rows() - voTo.rows()))
			Suitable = i;
		__generateResultImage(InputPyramid[i], "../TestData/Test019_Model/Input/Input" + std::to_string(i) + ".png");
	}*/
	//auto Input = InputPyramid.front();
	auto Input = InputPyramid[Suitable];
	//auto Input = vFrom;

	//neighbor mask
	const int KernelOffset = 2; 
	const int KernelWidth = KernelOffset * 2 + 1;
	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> NeighborMask(KernelWidth, KernelWidth);
	NeighborMask.setConstant(0);
	for (int i = -KernelOffset; i <= KernelOffset; ++i)
		for (int k = -KernelOffset; k <= KernelOffset; ++k)
			if (i < 0 || (i == 0 && k < 0))
				NeighborMask(i + KernelOffset, k + KernelOffset) = 1;

	for (Eigen::Index RowId = 0; RowId < voTo.rows(); ++RowId)
		for (Eigen::Index ColId = 0; ColId < voTo.cols(); ++ColId)
		{
			auto& Item = voTo.coeffRef(RowId, ColId);
			if (!__isAvailable(Item))
			{
				auto Feature = __buildFeatureWithNeighborMask(voTo, RowId, ColId, NeighborMask);

				float MinDistance = FLT_MAX;
				std::pair<Eigen::Index, Eigen::Index> MinPos;
				for (int i = 0; i < Input.rows(); i++)
					for (int k = 0; k < Input.cols(); k++)
					{
						auto Distance = __computeDistance(Feature, __buildFeatureWithNeighborMask(Input, i, k, NeighborMask));
						if (MinDistance > Distance)
						{
							MinDistance = Distance;
							MinPos = { i, k };
						}
					}

				Item = Input(MinPos.first, MinPos.second);
				//std::cout << _FORMAT_STR3("%1% %2% %3%\n", Input(MinPos.first, MinPos.second).row(0), Input(MinPos.first, MinPos.second).row(1), Input(MinPos.first, MinPos.second).row(2));
			}
		}
}

//*****************************************************************
//FUNCTION: 
template <typename Scalar_t, unsigned Channel>
void CTreeBasedTextureSynthesizer<Scalar_t, Channel>::__synthesizeTexture(int vLayer, int vGeneration)
{
	auto& Texture = m_Cache[vLayer][vGeneration];
	const auto Height = Texture.rows();
	const auto Width = Texture.cols();

	static tbb::affinity_partitioner Ap;
	tbb::parallel_for(tbb::blocked_range<size_t>(0, Height * Width),
		[&](const auto& vRange)
		{
			for (size_t i = vRange.begin(); i != vRange.end(); ++i)
			{
				auto& Item = Texture.coeffRef(i);
				if (!__isAvailable(Item))
					Item = __findNearestValue(vLayer, __buildOutputFeatureAt(vLayer, vGeneration, i % Height, i / Height));
			}
		}, Ap
	);
}

//*****************************************************************
//FUNCTION: 
template <typename Scalar_t, unsigned Channel>
auto CTreeBasedTextureSynthesizer<Scalar_t, Channel>::__buildOutputFeatureAt(int vLayer, int vGeneration, Eigen::Index vRowId, Eigen::Index vColId) const -> Feature_t
{
	return __buildFeatureAt(m_Cache[vLayer][vGeneration - 1], vRowId, vColId);
}

//*****************************************************************
//FUNCTION: 
template <typename Scalar_t, unsigned Channel>
auto CTreeBasedTextureSynthesizer<Scalar_t, Channel>::__buildFeatureAt(const Texture_t& vTexture, Eigen::Index vRowId, Eigen::Index vColId) const->Feature_t
{
	Eigen::Matrix<Scalar_t, Channel, Eigen::Dynamic> Feature(Channel, m_NeighborOffset.size());
	for (Eigen::Index It = 0; auto [RowOffset, ColOffset] : m_NeighborOffset)
	{
		auto RowIdWithOffset = __wrap(vTexture.rows(), vRowId + RowOffset);
		auto ColIdWithOffset = __wrap(vTexture.cols(), vColId + ColOffset);
		Feature.col(It++) = vTexture.coeff(RowIdWithOffset, ColIdWithOffset);
	}	
	return Eigen::Map<Feature_t>(Feature.data(), Feature.size());
}

//*****************************************************************
//FUNCTION: 
template <typename Scalar_t, unsigned Channel>
auto CTreeBasedTextureSynthesizer<Scalar_t, Channel>::__buildFeatureWithNeighborMask(const Texture_t& vTexture, Eigen::Index vRowId, Eigen::Index vColId, const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>& vMask) const->Feature_t
{
	const int KernelOffset = vMask.cols() / 2;
	Eigen::Matrix<Scalar_t, Channel, Eigen::Dynamic> Feature;
	std::vector<Eigen::Matrix<Scalar_t, Channel, 1>> FeatureCols;

	for (int i = -KernelOffset; i <= KernelOffset; ++i)
		for (int k = -KernelOffset; k <= KernelOffset; ++k)
			if (vMask(i + KernelOffset, k + KernelOffset) != 0)
			{
				auto RowIdWithOffset = __wrap(vTexture.rows(), vRowId + i);
				auto ColIdWithOffset = __wrap(vTexture.cols(), vColId + k);
				FeatureCols.push_back(vTexture(RowIdWithOffset, ColIdWithOffset));
			}

	Feature.resize(Channel, FeatureCols.size());

	for (auto It = 0; auto& Col : FeatureCols)
		Feature.col(It++) = Col;

	auto Map = Eigen::Map<Feature_t>(Feature.data(), Feature.size());
	return Eigen::Map<Feature_t>(Feature.data(), Feature.size());
}

//*****************************************************************
//FUNCTION: 
template <typename Scalar_t, unsigned Channel>
auto CTreeBasedTextureSynthesizer<Scalar_t, Channel>::__findNearestValue(int vLayer, const Feature_t& vFeature) const -> Color_t
{	
	auto Feature = vFeature;
	flann::Matrix Query(Feature.data(), 1, Feature.cols());
	flann::Matrix Index(new size_t, 1, 1);
	flann::Matrix Distance(new float, 1, 1);
	std::get<0>(m_SearchSet[vLayer])->knnSearch(Query, Index, Distance, 1, {});
	auto NearestValue = std::get<2>(m_SearchSet[vLayer]).row(Index[0][0]);

	delete[] Index.ptr();
	delete[] Distance.ptr();

	return NearestValue;
}

//*****************************************************************
//FUNCTION: 
template <typename Scalar_t, unsigned Channel>
void CTreeBasedTextureSynthesizer<Scalar_t, Channel>::__generateResultImage(const Texture_t& vTexture, const std::string& vOutputImagePath) const
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
