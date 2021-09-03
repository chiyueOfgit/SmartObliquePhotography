#include "pch.h"
#include "RayCastingBaker.h"

using namespace hiveObliquePhotography::SceneReconstruction;

_REGISTER_NORMAL_PRODUCT(CRayCastingBaker, KEYWORD::RAYCASTING_TEXTUREBAKER)

//*****************************************************************
//FUNCTION: 
hiveObliquePhotography::CImage<Eigen::Vector3i> CRayCastingBaker::bakeTexture(PointCloud_t::Ptr vPointCloud)
{
	m_pCloud = vPointCloud;

	return {};
}

//*****************************************************************
//FUNCTION: 
std::vector<STexelInfo> CRayCastingBaker::findTexelsPerFace(const SFace& vFace, Eigen::Vector2i vResolution)
{
	std::vector<STexelInfo> TexelInfos;

	auto& VertexA = m_Mesh.m_Vertices[vFace.a];
	auto& VertexB = m_Mesh.m_Vertices[vFace.b];
	auto& VertexC = m_Mesh.m_Vertices[vFace.c];

	//得到平面包围盒
	Eigen::Vector2f Min{ FLT_MAX, FLT_MAX };
	Eigen::Vector2f Max{ -FLT_MAX, -FLT_MAX };
	for (int i = 0; i < 3; i++)	//a, b, c
	{
		auto VertexUV = m_Mesh.m_Vertices[vFace[i]].uv();
		for (int i = 0; i < 2; i++)
		{
			if (Min.data()[i] < VertexUV.data()[i])
				Min.data()[i] = VertexUV.data()[i];
			if (Max.data()[i] > VertexUV.data()[i])
				Max.data()[i] = VertexUV.data()[i];
		}

	}
	
	for (int X = Min.x() * vResolution.x(); X < Max.x() * vResolution.x(); X++)
		for (int Y = Min.y() * vResolution.y(); Y < Max.y() * vResolution.y(); Y++)
		{
			//暂时单点采样, 以后可以纹素内多重采样
			Eigen::Vector2f CenterUV = { (X + 0.5f) / vResolution.x(), (Y + 0.5f) / vResolution.y() };

			if (__isPointInTriangle(CenterUV, VertexA.uv(), VertexB.uv(), VertexC.uv()))
			{
				STexelInfo Info;
				Info.OriginFace = vFace;
				Info.TexelPos = { X, Y };
				auto Barycentric = __calcBarycentric(CenterUV, VertexA.uv(), VertexB.uv(), VertexC.uv());
				auto PosInWorld = Barycentric.x() * VertexA.xyz() + Barycentric.y() * VertexB.xyz() + Barycentric.z() * VertexC.xyz();
				Info.TexelPosInWorld = PosInWorld;

				TexelInfos.push_back(Info);
			}
		}

	return TexelInfos;
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
	auto RayOrigin = vInfo.TexelPosInWorld;
	auto RayDirection = __calcRayDirection(vInfo);

	//决定每采样点权重
	std::vector<std::pair<std::size_t, float>> CandidateWeights;
	for (int i = 0; i < vCandidates.size(); i++)
	{
		auto VectorCandidate = vCandidates[i].Pos - RayOrigin;
		float DistanceToTexel = VectorCandidate.norm();	//纵向距离
		float DistanceToRay = VectorCandidate.cross(RayDirection).norm();	//横向距离

		float Weight = 1 / (DistanceToTexel * DistanceToRay);

		CandidateWeights.push_back({ i, Weight });
	}

	//决定多少采样点混合, 先固定为3, 有特别多可以考虑的
	const int NumBlend = 3;
	Eigen::Vector3i WeightedColor{ 0, 0, 0 };
	float SumWeight = 0.0f;
	std::sort(CandidateWeights.begin(), CandidateWeights.end(), [](std::pair<std::size_t, float> vLeft, std::pair<std::size_t, float> vRight) { return vLeft.second > vLeft.second; });
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
bool CRayCastingBaker::__isPointInTriangle(const Eigen::Vector2f& vPoint, const Eigen::Vector2f& vA, const Eigen::Vector2f& vB, const Eigen::Vector2f& vC)
{
	return true;
}

//*****************************************************************
//FUNCTION: 
Eigen::Vector3f CRayCastingBaker::__calcBarycentric(const Eigen::Vector2f& vPoint, const Eigen::Vector2f& vA, const Eigen::Vector2f& vB, const Eigen::Vector2f& vC)
{
	Eigen::Vector3f Temp[2];
	for (int i = 2; i--; )
	{
		Temp[i].x() = vC.data()[i] - vA.data()[i];
		Temp[i].y() = vB.data()[i] - vA.data()[i];
		Temp[i].z() = vA.data()[i] - vPoint.data()[i];
	}
	Eigen::Vector3f TempVec = Temp[0].cross(Temp[1]);
	return { 1.0f - (TempVec.x() + TempVec.y()) / TempVec.z(), TempVec.y() / TempVec.z(), TempVec.x() / TempVec.z() };
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


