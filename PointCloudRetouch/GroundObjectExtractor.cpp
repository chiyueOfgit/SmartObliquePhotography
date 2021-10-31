#include "pch.h"
#include "GroundObjectExtractor.h"
using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_NORMAL_PRODUCT(CGroundObjectExtractor, KEYWORD::GROUND_OBJECT_EXTRACTOR)

//*****************************************************************
//FUNCTION:
void CGroundObjectExtractor::runV(pcl::Indices& voObjectIndices, Eigen::Vector2i& vResolution)
{
	_ASSERTE((vResolution.array() > 0).all());
	CImage<std::array<int, 1>> ElevationMap = __generateElevationMap(vResolution);
	__extractObjectIndices(ElevationMap, voObjectIndices);
}

//*****************************************************************
//FUNCTION:
hiveObliquePhotography::CImage<std::array<int, 1>> CGroundObjectExtractor::__generateElevationMap(Eigen::Vector2i& vResolution)
{
	CImage<std::array<int, 1>> ResultImage;
	auto pManager = CPointCloudRetouchManager::getInstance();
	std::vector<pcl::index_t> Indices;
	Eigen::Matrix<std::array<int, 1>, -1, -1> Texture(vResolution.y(), vResolution.x());
	auto Box = pManager->getScene().getBoundingBox(Indices);
	Eigen::Vector2f Offset{ (Box.second - Box.first).x() / vResolution.x(),(Box.second - Box.first).y() / vResolution.y() };
	Eigen::Vector2f HeightRange{ Box.first.z(), Box.second.z() };

	std::vector<std::vector<float>> HeightSet(vResolution.y(), std::vector<float>(vResolution.x(), -FLT_MAX));
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
void CGroundObjectExtractor::__extractObjectIndices(CImage<std::array<int, 1>>& vElevationMap, pcl::Indices& voIndices)
{
	
}

//*****************************************************************
//FUNCTION:
void CGroundObjectExtractor::__calcAreaElevation(Eigen::Vector2f& vMinCoord, Eigen::Vector2f& vOffset, std::vector<std::vector<float>>& vioHeightSet)
{
	auto Scene = CPointCloudRetouchManager::getInstance()->getScene();
	for(int j = 0; j < Scene.getNumPoint(); j++)
	{
		auto Position = Scene.getPositionAt(j);
		
		int Row = (Position.y() - vMinCoord.y()) / vOffset.y();
		int Col = (Position.x() - vMinCoord.x()) / vOffset.x();
		if (Row == vioHeightSet.size()) Row--;
		if (Col == vioHeightSet[0].size()) Col--;
		
		if (Position.z() > vioHeightSet[Row][Col])
			vioHeightSet[Row][Col] = Position.z();
	}
}

//*****************************************************************
//FUNCTION:
std::array<int, 1> CGroundObjectExtractor::__transElevation2Color(float vElevation, float vHeightDiff)
{
	int Color;
	auto Percentage = vElevation / vHeightDiff;
	Color = Percentage * 255;
	return { Color };
}