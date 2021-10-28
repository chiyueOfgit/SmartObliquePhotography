#include "pch.h"
#include "GroundObjectExtractor.h"
using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CGroundObjectExtractor, KEYWORD::GROUND_OBJECT_EXTRACTOR)

//*****************************************************************
//FUNCTION:
void CGroundObjectExtractor::runV(pcl::Indices& voObjectIndices, Eigen::Vector2i& vResolution)
{
	_ASSERTE((vResolution.array() > 0).all());
	CImage<std::array<int, 3>> ElevationMap = __generateElevationMap(vResolution);
	__extractObjectIndices(ElevationMap, voObjectIndices);
}

//*****************************************************************
//FUNCTION:
hiveObliquePhotography::CImage<std::array<int, 3>> CGroundObjectExtractor::__generateElevationMap(Eigen::Vector2i& vResolution)
{
	CImage<std::array<int, 3>> ResultImage;
	return ResultImage;
}

//*****************************************************************
//FUNCTION:
void CGroundObjectExtractor::__extractObjectIndices(CImage<std::array<int, 3>>& vElevationMap, pcl::Indices& voIndices)
{
	
}