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
	std::vector<STexelInfo> ResultSet;
	
	auto PointA = m_Mesh.m_Vertices[vFace.a];
	auto PointB = m_Mesh.m_Vertices[vFace.b];
	auto PointC = m_Mesh.m_Vertices[vFace.c];
	auto Box = __calcBoxInTextureCoord(PointA.uv(), PointB.uv(), PointC.uv());
	/*int U = 0;
	int V = 0;*/
	//Eigen::Vector2f Offset{1 / vResolution[0], 1 / vResolution[1] };
	/*while (U < Box.first[0] * vResolution[0])
		U ++;
	while (V < Box.first[1])
		V += Offset[1];*/

	for(int U = Box.first[0] * vResolution[0] - 1; U < Box.second[0] * vResolution[0] + 1; U ++)
	{
		for(int V = Box.first[1] * vResolution[1]; V < Box.second[1] * vResolution[1] + 1; V ++)
		{
			auto BarycentricCoord = __calcBarycentricCoord(PointA.uv(), PointB.uv(), PointC.uv(), { (float)U / vResolution[0],(float)V / vResolution[1] });
			if( std::get<0>(BarycentricCoord) < 0 || std::get<1>(BarycentricCoord) < 0 || std::get<2>(BarycentricCoord) < 0 )
				continue;
			else
			{
				auto WorldCoord = __calcTexelCoordInWorld(PointA.xyz(), PointB.xyz(), PointC.xyz(), BarycentricCoord);
				STexelInfo TexelInfo({ U,V }, WorldCoord, vFace);
				ResultSet.push_back(TexelInfo);
			}
		}
	}
	return ResultSet;
}

//*****************************************************************
//FUNCTION: 
std::vector<SCandidateInfo> CRayCastingBaker::executeIntersection(const STexelInfo& vInfo)
{
	return {};
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


std::pair<Eigen::Vector2f, Eigen::Vector2f> CRayCastingBaker::__calcBoxInTextureCoord(const Eigen::Vector2f& vPointA, const Eigen::Vector2f& vPointB, const Eigen::Vector2f& vPointC)
{
	Eigen::Vector2f Min{ FLT_MAX, FLT_MAX };
	Eigen::Vector2f Max{ -FLT_MAX, -FLT_MAX };
	auto update = [&](const Eigen::Vector2f& vPos)
	{
		for (int i = 0; i < 2; i++)
		{
			if (vPos.data()[i] < Min.data()[i])
				Min.data()[i] = vPos.data()[i];
			if (vPos.data()[i] > Max.data()[i])
				Max.data()[i] = vPos.data()[i];
		}
	};
	update(vPointA);
	update(vPointB);
	update(vPointC);
	return { Min , Max };
}

std::tuple<float, float, float> CRayCastingBaker::__calcBarycentricCoord(const Eigen::Vector2f& vPointA, const Eigen::Vector2f& vPointB, const Eigen::Vector2f& vPointC, const Eigen::Vector2f& vPointP)
{
	auto A = vPointA - vPointC;
	auto B = vPointB - vPointC;
	auto C = vPointP - vPointC;

	float CoeffA = (C.x() * B.y() - C.y() * B.x()) / (A.x() * B.y() - A.y() * B.x());
	float CoeffB = (C.x() * A.y() - C.y() * A.x()) / (B.x() * A.y() - B.y() * A.x());
	float CoeffC = 1 - CoeffA - CoeffB;
	
	return { CoeffA, CoeffB, CoeffC };
}

Eigen::Vector3f CRayCastingBaker::__calcTexelCoordInWorld(const Eigen::Vector3f& vPointA, const Eigen::Vector3f& vPointB, const Eigen::Vector3f& vPointC, const std::tuple<float, float, float>& vBarycentricCoord)
{
	auto X = vPointA[0] * std::get<0>(vBarycentricCoord) + vPointB[0] * std::get<1>(vBarycentricCoord) + vPointC[0] * std::get<2>(vBarycentricCoord);
	auto Y = vPointA[1] * std::get<0>(vBarycentricCoord) + vPointB[1] * std::get<1>(vBarycentricCoord) + vPointC[1] * std::get<2>(vBarycentricCoord);
	auto Z = vPointA[2] * std::get<0>(vBarycentricCoord) + vPointB[2] * std::get<1>(vBarycentricCoord) + vPointC[2] * std::get<2>(vBarycentricCoord);
	
	return { X, Y, Z };
}