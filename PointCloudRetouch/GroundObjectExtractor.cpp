#include "pch.h"
#include "GroundObjectExtractor.h"
#include "ElevationMapGenerator.h"

#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_NORMAL_PRODUCT(CGroundObjectExtractor, KEYWORD::GROUND_OBJECT_EXTRACTOR)

void saveTexture(const std::string& vPath, const hiveObliquePhotography::CImage<float>& vTexture, bool vIsReverse)
{
	const auto Width = vTexture.getWidth();
	const auto Height = vTexture.getHeight();
	const auto BytesPerPixel = 1;
	auto ResultImage = new unsigned char[Width * Height * BytesPerPixel];
	for (auto i = 0; i < Height; i++)
		for (auto k = 0; k < Width; k++)
		{
			auto I = i;
			if (vIsReverse)
				I = Height - 1 - I;
			auto Offset = (I * Width + k) * BytesPerPixel;
			ResultImage[Offset] = vTexture.getColor(i, k);
			//ResultImage[Offset + 1] = vTexture.getColor(i, k)[1];
			//ResultImage[Offset + 2] = vTexture.getColor(i, k)[2];
		}

	stbi_write_png(vPath.c_str(), Width, Height, BytesPerPixel, ResultImage, 0);
	stbi_image_free(ResultImage);
}

//*****************************************************************
//FUNCTION:
void CGroundObjectExtractor::runV(pcl::Indices& voObjectIndices, std::vector<std::vector<pcl::index_t>>& voEdgeIndices, const Eigen::Vector2i& vResolution)
{
	_ASSERTE((vResolution.array() > 0).all());
	
	std::vector<pcl::index_t> PointIndexSet;
	std::size_t NumPoint = CPointCloudRetouchManager::getInstance()->getScene().getNumPoint();
	for (pcl::index_t i = 0; i < NumPoint; i++)
		PointIndexSet.push_back(i);

	CElevationMapGenerator ElevationMapGenerator;
	ElevationMapGenerator.execute(vResolution, PointIndexSet);
	CImage<float> ElevationMap;
	ElevationMapGenerator.dumpElevationMap(ElevationMap);
	ElevationMapGenerator.dumpPointDistributionSet(m_PointDistributionSet);

	saveTexture("ElevatioMap.png", ElevationMap, false);

	__extractObjectIndices(ElevationMap, voObjectIndices, voEdgeIndices);
	_ASSERTE(!voObjectIndices.empty());
}

//*****************************************************************
//FUNCTION:
void CGroundObjectExtractor::__extractObjectIndices(const CImage<float>& vElevationMap, pcl::Indices& voIndices, std::vector<std::vector<pcl::index_t>>& voEdgeIndices)
{
	auto ExtractedImage = __generateMaskByGrowing(vElevationMap, 4);
	__extractObjectByMask(vElevationMap, ExtractedImage);
	saveTexture("ExtractedImage.png", ExtractedImage, false);

	auto GroundEdgeImage = __extractGroundEdgeImage(ExtractedImage);
	saveTexture("GroundEdgeImage.png", GroundEdgeImage, false);

	__map2Cloud(ExtractedImage, voIndices, false);
	auto EdgeSet = __divide2EdgeSet(GroundEdgeImage);
	__map2Cloud(ExtractedImage, voEdgeIndices, EdgeSet);
}

//*****************************************************************
//FUNCTION:
hiveObliquePhotography::CImage<float> CGroundObjectExtractor::__generateMaskByGrowing(const CImage<float>& vOriginImage, int vThreshold)
{
	Eigen::Vector2i CurrentSeed = __findStartPoint(vOriginImage);
	Eigen::Vector2i NeighborSeed;
	float Flag;
	float SrcValue = 0.0f;
	float CurValue = 0.0f;
	int Height = vOriginImage.getHeight();
	int Width = vOriginImage.getWidth();

	Eigen::Matrix<float, -1, -1> BlackSet(Height, Width);
	for (int i = 0; i < Width; i++)
		for (int k = 0; k < Height; k++)
			BlackSet(k, i) = { 0 };
	CImage<float> MaskImage;
	MaskImage.fillColor(Height, Width, BlackSet.data());

	int Direction[8][2] = { {-1,-1}, {0,-1}, {1,-1}, {1,0}, {1,1}, {0,1}, {-1,1}, {-1,0} };
	std::vector<Eigen::Vector2i> SeedStack;
	SeedStack.push_back(CurrentSeed);
	MaskImage.fetchColor(CurrentSeed.y(), CurrentSeed.x()) = { 255 };

	while (!SeedStack.empty())
	{
		CurrentSeed = SeedStack.back();
		SeedStack.pop_back();
		SrcValue = vOriginImage.getColor(CurrentSeed.y(), CurrentSeed.x());
		for (int i = 0; i < 8; i++)
		{
			NeighborSeed.x() = CurrentSeed.x() + Direction[i][0];
			NeighborSeed.y() = CurrentSeed.y() + Direction[i][1];
			if (NeighborSeed.x() < 0 || NeighborSeed.y() < 0 || NeighborSeed.x() > (Width - 1) || (NeighborSeed.y() > Height - 1))
				continue;
			Flag = MaskImage.getColor(NeighborSeed.y(), NeighborSeed.x());
			if (Flag == 0)
			{
				CurValue = vOriginImage.getColor(NeighborSeed.y(), NeighborSeed.x());
				if (abs(SrcValue - CurValue) < vThreshold)
				{
					MaskImage.fetchColor(NeighborSeed.y(), NeighborSeed.x()) = { 255 };
					SeedStack.push_back(NeighborSeed);
				}
			}
		}
	}
	return MaskImage;
}

//*****************************************************************
//FUNCTION:
Eigen::Vector2i CGroundObjectExtractor::__findStartPoint(const CImage<float>& vImage)
{
	std::vector<int> Hist(256, 0);
	int StartColor = 0;
	Eigen::Vector2i LowestPosition;
	for (int i = 0; i < vImage.getWidth(); i++)
		for (int k = 0; k < vImage.getHeight(); k++)
			Hist[vImage.getColor(k, i)]++;
	for (; StartColor < 252; StartColor++)
		if (Hist[StartColor] > 100 && Hist[StartColor + 1] > 100 && Hist[StartColor + 2] > 100)
			break;
	for (int i = 0; i < vImage.getWidth(); i++)
		for (int k = 0; k < vImage.getHeight(); k++)
		{
			if (vImage.getColor(k, i) == StartColor)
			{
				LowestPosition.x() = i;
				LowestPosition.y() = k;
			}
		}
			
	return LowestPosition;
}

//*****************************************************************
//FUNCTION:
void CGroundObjectExtractor::__extractObjectByMask(const CImage<float>& vOriginImage, CImage<float>& vioMaskImage)
{
	for (int i = 0; i < vOriginImage.getWidth(); i++)
	{
		for (int k = 0; k < vOriginImage.getHeight(); k++)
		{
			if (vioMaskImage.getColor(k, i) == 0)
				vioMaskImage.fetchColor(k, i) = vOriginImage.getColor(k, i);
			else
				vioMaskImage.fetchColor(k, i) = 0; 
		}
	}
}

//*****************************************************************
//FUNCTION:
void CGroundObjectExtractor::__map2Cloud(const CImage<float>& vTexture, std::vector<pcl::index_t>& voCandidates, bool vIfObject)
{
	auto pManager = CPointCloudRetouchManager::getInstance();
	std::vector<pcl::index_t> Indices;
	auto Box = pManager->getScene().getBoundingBox(Indices);

	if (m_PointDistributionSet.empty())
	{
		// TODO:COPY-PASTE
		Eigen::Vector2f Offset{ (Box.second - Box.first).x() / vTexture.getWidth(),(Box.second - Box.first).y() / vTexture.getHeight() };
		Eigen::Vector2f HeightRange{ Box.first.z(), Box.second.z() };

		std::vector<std::vector<float>> HeightSet(vTexture.getHeight(), std::vector<float>(vTexture.getWidth(), HeightRange.x()));
		Eigen::Vector2f MinXY{ Box.first.x(),Box.first.y() };
	}

	for (int i = 0; i < vTexture.getHeight(); i++)
		for (int k = 0; k < vTexture.getWidth(); k++)
		{
			if (!(vIfObject * vTexture.getColor(i, k)) && (vIfObject + vTexture.getColor(i, k)))
				continue;

			auto Scene = CPointCloudRetouchManager::getInstance()->getScene();

			if (vIfObject)
			{
				Eigen::Vector2f ZRange{ static_cast<float>(vTexture.getColor(i, k)) / 255 * (Box.second - Box.first).z() + Box.first.z(), static_cast<float>(vTexture.getColor(i,k) + 1) / 255 * (Box.second - Box.first).z() + Box.first.z() };

				for (auto PointIndex : m_PointDistributionSet[i][k])
				{
					auto Position = Scene.getPositionAt(PointIndex);
					if ((Position.z() - ZRange.x()) * (Position.z() - ZRange.y()) <= 0)
						voCandidates.push_back(PointIndex);
				}
			}
			else
				for (auto PointIndex : m_PointDistributionSet[i][k])
					voCandidates.push_back(PointIndex);
		}
}

//*****************************************************************
//FUNCTION:
void CGroundObjectExtractor::__map2Cloud(const CImage<float>& vTexture, std::vector<std::vector<pcl::index_t>>& voEdgeIndices, const std::vector<std::vector<Eigen::Vector2i>>& vEdgeSet)
{
	// TODO:COPY-PASTE
	if (voEdgeIndices.size())
		voEdgeIndices.clear();
	auto pManager = CPointCloudRetouchManager::getInstance();
	std::vector<pcl::index_t> Indices;
	auto Box = pManager->getScene().getBoundingBox(Indices);

	if (m_PointDistributionSet.empty())
	{
		// TODO:COPY-PASTE
		Eigen::Vector2f Offset{ (Box.second - Box.first).x() / vTexture.getWidth(),(Box.second - Box.first).y() / vTexture.getHeight() };
		Eigen::Vector2f HeightRange{ Box.first.z(), Box.second.z() };

		std::vector<std::vector<float>> HeightSet(vTexture.getHeight(), std::vector<float>(vTexture.getWidth(), HeightRange.x()));
		Eigen::Vector2f MinXY{ Box.first.x(),Box.first.y() };
		//__calcAreaElevation(MinXY, Offset, HeightSet);
	}

	for (auto Edge : vEdgeSet)
	{
		std::vector<pcl::index_t> EdgePointSet;
		for (auto EdgePoint : Edge)
			for (auto PointIndex : m_PointDistributionSet[EdgePoint[1]][EdgePoint[0]])
				EdgePointSet.push_back(PointIndex);
		voEdgeIndices.push_back(EdgePointSet);
	}
}

//*****************************************************************
//FUNCTION:
hiveObliquePhotography::CImage<float> CGroundObjectExtractor::__extractGroundEdgeImage(const CImage<float>& vExtractedImage)
{
    auto Width = vExtractedImage.getWidth();
	auto Height = vExtractedImage.getHeight();
	Eigen::Matrix<float, -1, -1> WhiteSet(Height, Width);
	for (int i = 0; i < Width; i++)
		for (int k = 0; k < Height; k++)
			WhiteSet(k, i) = { 255 };
	CImage<float> GroundEdgeImage;
	GroundEdgeImage.fillColor(Height, Width, WhiteSet.data());

	int Direction[8][2] = { {-1,-1}, {0,-1}, {1,-1}, {1,0}, {1,1}, {0,1}, {-1,1}, {-1,0} };
	for (int i = 0; i < Width; i++)
	{
		for (int k = 0; k < Height; k++)
		{
			Eigen::Vector2i CurrentPixel{ i,k };
			int IsEdge = 0;
			int NeighborNum = 8;
			for (int m = 0; m < 8; m++)
			{
				Eigen::Vector2i NeighborPixel;
				NeighborPixel.x() = CurrentPixel.x() + Direction[m][0];
				NeighborPixel.y() = CurrentPixel.y() + Direction[m][1];
				if (NeighborPixel.x() < 0 || NeighborPixel.y() < 0 || NeighborPixel.x() > (Width - 1) || (NeighborPixel.y() > Height - 1))
				{
					NeighborNum--;
					continue;
				}
				auto Flag = vExtractedImage.getColor(NeighborPixel.y(), NeighborPixel.x());
				if (Flag != 0)
					IsEdge ++;
			}
			if (IsEdge != NeighborNum && IsEdge != 0)
				GroundEdgeImage.fetchColor(k, i) = { 0 };
		}
	}
	return GroundEdgeImage;
}

//*****************************************************************
//FUNCTION:
std::vector<std::vector<Eigen::Vector2i>> CGroundObjectExtractor::__divide2EdgeSet(const CImage<float>& vEdgeImage)
{
	std::vector<std::vector<Eigen::Vector2i>> OutputEdgeSet;
	auto TempImage = vEdgeImage;
	Eigen::Vector2i CurrentSeed;
	Eigen::Vector2i NeighborSeed;
	int Height = TempImage.getHeight();
	int Width = TempImage.getWidth();
	int Direction[8][2] = { {-1,-1}, {0,-1}, {1,-1}, {1,0}, {1,1}, {0,1}, {-1,1}, {-1,0} };

	while(1)
	{
		if(!__findBlackPoint(TempImage,CurrentSeed))
			break;
		std::vector<Eigen::Vector2i> SeedStack;
		SeedStack.push_back(CurrentSeed);
		std::vector<Eigen::Vector2i> EdgeSet;
	
		while (!SeedStack.empty())
		{
			CurrentSeed = SeedStack.back();
			SeedStack.pop_back();
			EdgeSet.push_back(CurrentSeed);
			/*if (EdgeSet.size() > 100)
				break;*/
			for (int i = 0; i < 8; i++)
			{
				NeighborSeed.x() = CurrentSeed.x() + Direction[i][0];
				NeighborSeed.y() = CurrentSeed.y() + Direction[i][1];
				if (NeighborSeed.x() < 0 || NeighborSeed.y() < 0 || NeighborSeed.x() > (Width - 1) || (NeighborSeed.y() > Height - 1))
					continue;
				auto Color = TempImage.getColor(NeighborSeed.y(), NeighborSeed.x());
				if (Color == 0)
				{
					SeedStack.push_back(NeighborSeed);
					TempImage.fetchColor(NeighborSeed.y(), NeighborSeed.x()) = 255;
				}
			}
		}
		if(EdgeSet.size() > 20)
		    OutputEdgeSet.push_back(EdgeSet);
	}
	return OutputEdgeSet;
}

//*****************************************************************
//FUNCTION:
bool CGroundObjectExtractor::__findBlackPoint(const CImage<float>& vImage, Eigen::Vector2i& voBlackPoint)
{
	for (int i = 0; i < vImage.getWidth(); i++)
	{
		for (int k = 0; k < vImage.getHeight(); k++)
		{
			if (vImage.getColor(k, i) == 0)
			{
				voBlackPoint.x() = i;
				voBlackPoint.y() = k;
				return true;
			}
		}
	}
	return false;
}
