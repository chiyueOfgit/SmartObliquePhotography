#include "pch.h"
#include "RayCastingBaker.h"

using namespace hiveObliquePhotography::SceneReconstruction;

_REGISTER_NORMAL_PRODUCT(CRayCastingBaker, KEYWORD::RAYCASTING_TEXTUREBAKER)

//*****************************************************************
//FUNCTION: 
hiveObliquePhotography::CImage<Eigen::Vector3i> CRayCastingBaker::bakeTexture(PointCloud_t::Ptr vPointCloud)
{
	return {};
}

//*****************************************************************
//FUNCTION: 
std::vector<STexelInfo> CRayCastingBaker::findTexelsPerFace(const SFace& vFace, Eigen::Vector2i vResolution)
{
	return {};
}

//*****************************************************************
//FUNCTION: 
std::vector<SCandidateInfo> CRayCastingBaker::executeIntersection(const STexelInfo& vInfo)
{
	return {};
}

//*****************************************************************
//FUNCTION: 
Eigen::Vector3i CRayCastingBaker::calcTexelColor(const std::vector<SCandidateInfo>& vCandidates)
{
	return {};
}
