#include "pch.h"
#include "GroundObjectExtractor.h"
#include "ElevationMapGenerator.h"
#include "Utility.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_NORMAL_PRODUCT(CGroundObjectExtractor, KEYWORD::GROUND_OBJECT_EXTRACTOR)

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
	Eigen::Vector2i CurrentSeed = __findStartPoint(vOriginImage, vThreshold);
	CImage<float> MaskImage;
	int Height = vOriginImage.getHeight();
	int Width = vOriginImage.getWidth();
	Eigen::MatrixXf BlackSet = Eigen::MatrixXf::Zero(Height, Width);
	int Direction[8][2] = { {-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0} };
	std::vector<Eigen::Vector2i> SeedStack;
	SeedStack.push_back(CurrentSeed);
	MaskImage.fillColor(Height, Width, BlackSet.data());
	MaskImage.fetchColor(CurrentSeed.y(), CurrentSeed.x()) = 255;

	while (!SeedStack.empty())
	{
		CurrentSeed = SeedStack.back();
		SeedStack.pop_back();
		float SrcValue = vOriginImage.getColor(CurrentSeed.y(), CurrentSeed.x());
		for (int i = 0; i < 8; i++)
		{
			Eigen::Vector2i NeighborSeed{ CurrentSeed.x() + Direction[i][0], CurrentSeed.y() + Direction[i][1] };
			if (NeighborSeed.x() < 0 || NeighborSeed.y() < 0 || NeighborSeed.x() > (Width - 1) || (NeighborSeed.y() > Height - 1))
				continue;

			if (MaskImage.getColor(NeighborSeed.y(), NeighborSeed.x()) == 0.0f)
				if (abs(SrcValue - vOriginImage.getColor(NeighborSeed.y(), NeighborSeed.x())) < vThreshold)
				{
					MaskImage.fetchColor(NeighborSeed.y(), NeighborSeed.x()) = 255;
					SeedStack.push_back(NeighborSeed);
				}
		}
	}
	return MaskImage;
}

//*****************************************************************
//FUNCTION:
Eigen::Vector2i CGroundObjectExtractor::__findStartPoint(const CImage<float>& vImage, int vThreshold)
{
	std::vector<int> Hist(256, 0);
	Eigen::Vector2i LowestPosition;
	for (int i = 0; i < vImage.getWidth(); i++)
		for (int k = 0; k < vImage.getHeight(); k++)
			Hist[static_cast<int>(vImage.getColor(k, i))]++;

	int StartColor = 0;
	for (; StartColor < 256 - vThreshold; StartColor++)
	{
		bool Continuity = true;
		for (int i = 0; i < vThreshold - 1; i++)
			if (Hist[StartColor + i] < 100)
			{
				Continuity = false;
				break;
			}
		if (Continuity)
			break;
	}
	
	for (int i = 0; i < vImage.getWidth(); i++)
		for (int k = 0; k < vImage.getHeight(); k++)
			if (static_cast<int>(vImage.getColor(k, i)) == StartColor)
			{
				LowestPosition.x() = i;
				LowestPosition.y() = k;
			}
			
	return LowestPosition;
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
	auto Scene = CPointCloudRetouchManager::getInstance()->getScene();
	auto Box = Scene.getBoundingBox(std::vector<pcl::index_t>());

	for (int i = 0; i < vTexture.getHeight(); i++)
		for (int k = 0; k < vTexture.getWidth(); k++)
		{
			if (!(vIfObject * vTexture.getColor(i, k)) && (vIfObject + vTexture.getColor(i, k)))
				continue;

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
	if (voEdgeIndices.size())
		voEdgeIndices.clear();

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
	Eigen::MatrixXf WhiteSet = Eigen::MatrixXf::Constant(Height, Width, 255.0f);
	CImage<float> GroundEdgeImage;
	GroundEdgeImage.fillColor(Height, Width, WhiteSet.data());

	int Direction[8][2] = { {-1,-1}, {0,-1}, {1,-1}, {1,0}, {1,1}, {0,1}, {-1,1}, {-1,0} };
	for (int i = 0; i < Width; i++)
		for (int k = 0; k < Height; k++)
		{
			Eigen::Vector2i CurrentPixel{ i, k };
			int NumEdge = 0;
			int NeighborNum = 8;
			for (int m = 0; m < 8; m++)
			{
				Eigen::Vector2i NeighborPixel{ CurrentPixel.x() + Direction[m][0], CurrentPixel.y() + Direction[m][1] };
				if (NeighborPixel.x() < 0 || NeighborPixel.y() < 0 || NeighborPixel.x() > (Width - 1) || (NeighborPixel.y() > Height - 1))
				{
					NeighborNum--;
					continue;
				}
				if (vExtractedImage.getColor(NeighborPixel.y(), NeighborPixel.x()) != 0)
					NumEdge++;
			}
			if (NumEdge != NeighborNum && NumEdge != 0)
				GroundEdgeImage.fetchColor(k, i) = 0;
		}
	return GroundEdgeImage;
}

//*****************************************************************
//FUNCTION:
std::vector<std::vector<Eigen::Vector2i>> CGroundObjectExtractor::__divide2EdgeSet(const CImage<float>& vEdgeImage)
{
	std::vector<std::vector<Eigen::Vector2i>> OutputEdgeSet;
	auto TempImage = vEdgeImage;
	int Direction[8][2] = { {-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0} };

	while(true)
	{
		Eigen::Vector2i CurrentSeed;
		if(!__findBlackPoint(TempImage, CurrentSeed))
			break;
		std::vector<Eigen::Vector2i> EdgeSet;
		std::vector<Eigen::Vector2i> SeedStack;
		SeedStack.push_back(CurrentSeed);
		
		while (!SeedStack.empty())
		{
			CurrentSeed = SeedStack.back();
			SeedStack.pop_back();
			EdgeSet.push_back(CurrentSeed);

			for (int i = 0; i < 8; i++)
			{
				Eigen::Vector2i NeighborSeed{ CurrentSeed.x() + Direction[i][0], CurrentSeed.y() + Direction[i][1] };

				if (NeighborSeed.x() < 0 || NeighborSeed.y() < 0 || NeighborSeed.x() > (TempImage.getWidth() - 1) || (NeighborSeed.y() > TempImage.getHeight() - 1))
					continue;
				if (TempImage.getColor(NeighborSeed.y(), NeighborSeed.x()) == 0)
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
