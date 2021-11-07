#include "pch.h"
#include "GroundObjectExtractor.h"

#include <flann/util/matrix.h>
using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_NORMAL_PRODUCT(CGroundObjectExtractor, KEYWORD::GROUND_OBJECT_EXTRACTOR)

#define Radius 0.3

//*****************************************************************
//FUNCTION:
void CGroundObjectExtractor::runV(pcl::Indices& voObjectIndices,const Eigen::Vector2i& vResolution)
{
	_ASSERTE((vResolution.array() > 0).all());
	CImage<std::array<int, 1>> ElevationMap = __generateElevationMap(vResolution);
	__extractObjectIndices(ElevationMap, voObjectIndices);
	_ASSERTE(!voObjectIndices.empty());
}

//*****************************************************************
//FUNCTION:
hiveObliquePhotography::CImage<std::array<int, 1>> CGroundObjectExtractor::__generateElevationMap(const Eigen::Vector2i& vResolution)
{
	CImage<std::array<int, 1>> ResultImage;
	auto pManager = CPointCloudRetouchManager::getInstance();
	std::vector<pcl::index_t> Indices;
	Eigen::Matrix<std::array<int, 1>, -1, -1> Texture(vResolution.y(), vResolution.x());
	auto Box = pManager->getScene().getBoundingBox(Indices);
	Eigen::Vector2f Offset{ (Box.second - Box.first).x() / vResolution.x(),(Box.second - Box.first).y() / vResolution.y() };
	Eigen::Vector2f HeightRange{ Box.first.z(), Box.second.z() };

	std::vector<std::vector<float>> HeightSet(vResolution.y(), std::vector<float>(vResolution.x(), HeightRange.x()));
	Eigen::Vector2f MinXY{ Box.first.x(),Box.first.y() };
	__calcAreaElevation(MinXY, Offset, HeightSet);
	
	for (int i = 0; i < vResolution.x(); i++)
	{
		for (int k = 0; k < vResolution.y(); k++)
		{
			auto Elevation = HeightSet[k][i];
			Texture(k, i) = __transElevation2Color(Elevation - HeightRange.x(), HeightRange.y() - HeightRange.x());
		}
	}

	ResultImage.fillColor(vResolution.y(), vResolution.x(), Texture.data());
	return ResultImage;
}

//*****************************************************************
//FUNCTION:
void CGroundObjectExtractor::__extractObjectIndices(const CImage<std::array<int, 1>>& vElevationMap, pcl::Indices& voIndices)
{
	auto ExtractedImage = __generateMaskByGrowing(vElevationMap, 4);
	__extractObjectByMask(vElevationMap, ExtractedImage);
	__map2Cloud(ExtractedImage, voIndices);
}

//*****************************************************************
//FUNCTION:
void CGroundObjectExtractor::__calcAreaElevation(const Eigen::Vector2f& vMinCoord, const Eigen::Vector2f& vOffset, std::vector<std::vector<float>>& vioHeightSet)
{
	auto Scene = CPointCloudRetouchManager::getInstance()->getScene();
	for(int j = 0; j < Scene.getNumPoint(); j++)
	{
		auto Position = Scene.getPositionAt(j);
		
		int RowBegin = (Position.y() - vMinCoord.y() - Radius) / vOffset.y();
		int RowEnd = (Position.y() - vMinCoord.y() + Radius) / vOffset.y();
		int ColBegin = (Position.x() - vMinCoord.x() - Radius) / vOffset.x();
		int ColEnd = (Position.x() - vMinCoord.x() + Radius) / vOffset.x();
		
		if (RowBegin > vioHeightSet.size()) RowBegin = vioHeightSet.size();
		if (RowEnd > vioHeightSet.size()) RowEnd = vioHeightSet.size();
		if (ColBegin > vioHeightSet[0].size()) ColBegin = vioHeightSet[0].size();
		if (ColEnd > vioHeightSet[0].size()) ColEnd = vioHeightSet[0].size();

		for (int i = ColBegin; i < ColEnd; i++)
		{
			for (int k = RowBegin; k < RowEnd; k++)
			{
				if (Position.z() > vioHeightSet[k][i])
			        vioHeightSet[k][i] = Position.z();
			}
		}
		
	}
}

bool confirmInRange(float vNumber, const Eigen::Vector2f& vRange)
{
	return (vNumber - vRange[0]) * (vNumber - vRange[1]) < 0;
}

//*****************************************************************
//FUNCTION:
std::array<int, 1> CGroundObjectExtractor::__transElevation2Color(float vElevation, float vHeightDiff)
{
	int Color;
	auto Percentage = vElevation / vHeightDiff;
	Color = Percentage * 255;
	return { Color };

	/*std::array Color = { 0, 0, 0 };
	auto Percentage = vElevation / vHeightDiff;
	switch (static_cast<int>(Percentage * 4))
	{
	case 0:
		Color[1] = Percentage * 1020;
		Color[2] = 255;
		break;
	case 1:
		Color[1] = 255;
		Color[2] = 255 - (Percentage - 0.25) * 1020;
		break;
	case 2:
		Color[0] = (Percentage - 0.5) * 1020;
		Color[1] = 255;
		break;
	case 3:
		Color[0] = 255;
		Color[1] = 255 - (Percentage - 0.75) * 1020;
		break;
	default:
		break;
	}

	return Color;*/

}

//*****************************************************************
//FUNCTION:
Eigen::Vector2i CGroundObjectExtractor::__findStartPoint(const CImage<std::array<int, 1>>& vImage)
{
	int Min = INT_MAX;
	Eigen::Vector2i LowestPosition;
	for (int i = 0; i < vImage.getWidth(); i++)
	{
		for (int k = 0; k < vImage.getHeight(); k++)
		{
			if(vImage.getColor(k,i)[0] < Min)
			{
				Min = vImage.getColor(k, i)[0];
				LowestPosition.x() = i;
				LowestPosition.y() = k;
			}
		}
	}
	return LowestPosition;
}

//*****************************************************************
//FUNCTION:
hiveObliquePhotography::CImage<std::array<int, 1>> CGroundObjectExtractor::__generateMaskByGrowing(const CImage<std::array<int, 1>>& vOriginImage, int vThreshold)
{

	Eigen::Vector2i CurrentSeed = __findStartPoint(vOriginImage);
	Eigen::Vector2i NeighborSeed;						
	std::array<int, 1> Flag;
	std::array<int, 1> SrcValue = {0};
	std::array<int, 1> CurValue = {0};  
	int Height = vOriginImage.getHeight();
	int Width = vOriginImage.getWidth();
	
	Eigen::Matrix<std::array<int, 1>, -1, -1> BlackSet(Height, Width);
	for (int i = 0; i < Width; i++)
		for (int k = 0; k < Height; k++)
			BlackSet(k, i) = {0};
	CImage<std::array<int, 1>> MaskImage;
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
		for (int i = 0; i < 9; ++i)
		{
			NeighborSeed.x() = CurrentSeed.x() + Direction[i][0];
			NeighborSeed.y() = CurrentSeed.y() + Direction[i][1];
			if (NeighborSeed.x() < 0 || NeighborSeed.y() < 0 || NeighborSeed.x() >(Width - 1) || (NeighborSeed.y() > Height - 1))
				continue;
			Flag = MaskImage.getColor(NeighborSeed.y(), NeighborSeed.x());
			if (Flag[0] == 0)					
			{
				CurValue = vOriginImage.getColor(NeighborSeed.y(), NeighborSeed.x());
				if (abs(SrcValue[0] - CurValue[0]) < vThreshold)					
				{
					MaskImage.fetchColor(NeighborSeed.y(), NeighborSeed.x()) = {255};
					SeedStack.push_back(NeighborSeed);
				}
			}
		}
	}
	return MaskImage;
}

//*****************************************************************
//FUNCTION:
void CGroundObjectExtractor::__extractObjectByMask(const CImage<std::array<int, 1>>& vOriginImage, CImage<std::array<int, 1>>& vioMaskImage)
{
	for (int i = 0; i < vOriginImage.getWidth(); i++)
	{
		for (int k = 0; k < vOriginImage.getHeight(); k++)
		{
			if (vioMaskImage.getColor(k, i)[0] == 0)
				vioMaskImage.fetchColor(k, i)[0] = vOriginImage.getColor(k, i)[0];
			else
				vioMaskImage.fetchColor(k, i)[0] = 0;
		}
	}
}

void CGroundObjectExtractor::__map2Cloud(const CImage<std::array<int, 1>>& vTexture, std::vector<pcl::index_t>& voCandidates)
{
	auto pManager = CPointCloudRetouchManager::getInstance();
	std::vector<pcl::index_t> Indices;
	auto Box = pManager->getScene().getBoundingBox(Indices);
	for (int i = 0; i < vTexture.getHeight(); i++)
		for (int k = 0; k < vTexture.getWidth(); k++)
		{
			if (!vTexture.getColor(i, k)[0])
				continue;

			Eigen::Vector2f XRange{ static_cast<float>(k) / vTexture.getWidth() * (Box.second - Box.first).x() + Box.first.x(), static_cast<float>(k + 1) / vTexture.getWidth() * (Box.second - Box.first).x() + Box.first.x() };
			Eigen::Vector2f YRange{ static_cast<float>(i) / vTexture.getHeight() * (Box.second - Box.first).y() + Box.first.y(), static_cast<float>(i + 1) / vTexture.getHeight() * (Box.second - Box.first).y() + Box.first.y() };
			Eigen::Vector2f ZRange{ static_cast<float>(vTexture.getColor(i, k)[0]) / 255 * (Box.second - Box.first).z() + Box.second.z(), static_cast<float>(vTexture.getColor(i,k)[0] + 1) / 255 * (Box.second - Box.first).z() + Box.second.z() };

			auto Scene = CPointCloudRetouchManager::getInstance()->getScene();
			for (int m = 0; m < Scene.getNumPoint(); m++)
			{
				auto Position = Scene.getPositionAt(m);
				if (confirmInRange(Position.x(), XRange) && confirmInRange(Position.y(), YRange) && confirmInRange(Position.z(), ZRange))
					voCandidates.push_back(m);
			}
		}
}