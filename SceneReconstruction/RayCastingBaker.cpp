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
	std::vector<SCandidateInfo> Candidates;

	auto RayOrigin = vInfo.TexelPosInWorld;
	auto RayDirection = __calcRayDirection(vInfo);

	auto CulledPoints = __cullPointsByRay(RayOrigin, RayDirection);

	for (auto Index : CulledPoints)
	{
		auto& TestPoint = m_pCloud->points[Index];
		Eigen::Vector3f SurfelPos{ TestPoint.x, TestPoint.y, TestPoint.z };
		Eigen::Vector3f SurfelNormal{ TestPoint.normal_x, TestPoint.normal_y, TestPoint.normal_z };
		float Depth = (SurfelPos - RayOrigin).dot(SurfelNormal) / RayDirection.dot(SurfelNormal);

		auto HitPos = RayOrigin + Depth * RayDirection;

		float DistanceToCenter = (HitPos - SurfelPos).norm();
		float DistanceToTexel = (HitPos - RayOrigin).norm();
		const float SurfelRadius = 10.0f;
		//距离反比的半径
		if (DistanceToCenter < SurfelRadius / DistanceToTexel)
			Candidates.push_back({ HitPos, Index });
	}

	return Candidates;
}

//*****************************************************************
//FUNCTION: 
Eigen::Vector3i CRayCastingBaker::calcTexelColor(const std::vector<SCandidateInfo>& vCandidates, const STexelInfo& vInfo)
{
	auto BeginPos = vInfo.TexelPosInWorld;
	auto RayDirection = __calcRayDirection(vInfo);

	//决定每采样点权重
	std::vector<std::pair<std::size_t, float>> CandidateWeights;
	for (int i = 0; i < vCandidates.size(); i++)
	{
		auto VectorCandidate = vCandidates[i].Pos - BeginPos;
		float DistanceToTexel = VectorCandidate.norm();	//纵向距离
		float DistanceToRay = VectorCandidate.cross(RayDirection).norm();	//横向距离

		float Weight = 1 / (DistanceToTexel * DistanceToRay);

		CandidateWeights.push_back({ i, Weight });
	}

	//决定多少采样点混合, 先固定为3, 有特别多可以考虑的
	const int NumBlend = 3;
	Eigen::Vector3i WeightedColor{ 0, 0, 0 };
	float SumWeight = 0.0f;
	auto pCompare = [](std::pair<std::size_t, float> vLeft, std::pair<std::size_t, float> vRight)
	{
		return vLeft.second > vLeft.second;
	};
	std::sort(CandidateWeights.begin(), CandidateWeights.end(), pCompare);
	for (int i = 0; i < NumBlend; i++)
	{
		auto& Point = m_pCloud->points[vCandidates[CandidateWeights[i].first].PointIndex];
		WeightedColor += (Eigen::Vector3i{ Point.r, Point.g, Point.b }.cast<float>() * CandidateWeights[i].second).cast<int>();
		SumWeight += CandidateWeights[i].second;
	}

	return (WeightedColor.cast<float>() / SumWeight).cast<int>();
}

//*****************************************************************
//FUNCTION: 
Eigen::Vector3f CRayCastingBaker::__calcRayDirection(const STexelInfo& vInfo)	//暂用那个面的法线
{
	auto PosA = m_Mesh.m_Vertices[vInfo.OriginFace.a].xyz();
	auto PosB = m_Mesh.m_Vertices[vInfo.OriginFace.b].xyz();
	auto PosC = m_Mesh.m_Vertices[vInfo.OriginFace.c].xyz();

	return ((PosB - PosA).cross(PosC - PosA)).normalized();
}

//*****************************************************************
//FUNCTION: 
std::vector<pcl::index_t> CRayCastingBaker::__cullPointsByRay(const Eigen::Vector3f& vRayBegin, const Eigen::Vector3f& vRayDirection)
{
	std::vector<pcl::index_t> PointIndices;

	//暂时不剔除
	for (int i = 0; i < m_pCloud->size(); i++)
	{

		PointIndices.push_back(i);
	}

	return PointIndices;
}


