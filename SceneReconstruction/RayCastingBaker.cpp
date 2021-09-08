#include "pch.h"
#include "RayCastingBaker.h"

using namespace hiveObliquePhotography::SceneReconstruction;

_REGISTER_NORMAL_PRODUCT(CRayCastingBaker, KEYWORD::RAYCASTING_TEXTUREBAKER)

const float SurfelRadius = 0.5f;	//magic raidus
const float DistanceThreshold = 0.2f;	//magic distance

//*****************************************************************
//FUNCTION: 
hiveObliquePhotography::CImage<Eigen::Vector3i> CRayCastingBaker::bakeTexture(PointCloud_t::Ptr vPointCloud, const Eigen::Vector2i& vResolution)
{
	m_pCloud = vPointCloud;
	__buildKdTree(m_pCloud);

	Eigen::Matrix<Eigen::Vector3i, -1, -1> Texture;
	Texture.resize(vResolution.y(), vResolution.x());
	CImage<Eigen::Vector3i> ResultTexture;

	/*for (auto& Face : m_Mesh.m_Faces)
	{
		auto TexelInfos = findTexelsPerFace(Face, vResolution);
		for (auto& TexelInfo : TexelInfos)
		{
			auto Candidates = executeIntersection(TexelInfo);
			auto TexelColor = calcTexelColor(Candidates, TexelInfo);
			Texture(TexelInfo.TexelPos.y(), TexelInfo.TexelPos.x()) = TexelColor;
		}
	}*/

	for (auto& Face : m_Mesh.m_Faces)
	{
		auto TexelSampleInfoSet = findSamplesPerFace(Face, vResolution);
		for (auto& PerTexel : TexelSampleInfoSet)
		{
			for (auto& SampleInfo : PerTexel.SamplePosSetInWorld)
			{
				auto Candidates = executeIntersection(SampleInfo);
				auto TexelColor = calcTexelColor(Candidates, SampleInfo);
			}
            //TODO：Texel混合颜色
			Texture(PerTexel.TexelPos.y(), PerTexel.TexelPos.x()) = TexelColor;
		}
	}
	ResultTexture.fillColor(vResolution.y(), vResolution.x(), Texture.data());
	return ResultTexture;
}

//*****************************************************************
//FUNCTION: 
std::vector<STexelInfo> CRayCastingBaker::findTexelsPerFace(const SFace& vFace, Eigen::Vector2i vResolution)
{
	std::vector<STexelInfo> ResultSet;
	
	auto& VertexA = m_Mesh.m_Vertices[vFace.a];
	auto& VertexB = m_Mesh.m_Vertices[vFace.b];
	auto& VertexC = m_Mesh.m_Vertices[vFace.c];

	auto Box = __calcBoxInTextureCoord(VertexA.uv(), VertexB.uv(), VertexC.uv());

	for (int X = Box.first.x() * vResolution.x() - 1; X < Box.second.x() * vResolution.x() + 1; X++)
		for (int Y = Box.first.y() * vResolution.y() - 1; Y < Box.second.y() * vResolution.y() + 1; Y++)
			if (X >= 0 && X < vResolution.x() && Y >= 0 && Y < vResolution.y())
			{
				Eigen::Vector2f CenterUV = { (X + 0.5f) / vResolution.x(), (Y + 0.5f) / vResolution.y() };

				auto BarycentricCoord = __calcBarycentricCoord(VertexA.uv(), VertexB.uv(), VertexC.uv(), CenterUV);
				if ((BarycentricCoord.array() >= 0).all())
				{
					auto WorldPos = __calcTexelPosInWorld(VertexA.xyz(), VertexB.xyz(), VertexC.xyz(), BarycentricCoord);
					ResultSet.emplace_back(Eigen::Vector2i{ X, Y }, WorldPos, vFace);
				}
			}

	return ResultSet;
}

std::vector<STexelSampleInfo> CRayCastingBaker::findSamplesPerFace(const SFace& vFace, Eigen::Vector2i vResolution)
{
	std::vector<STexelSampleInfo> ResultSet;

	/*auto& VertexA = m_Mesh.m_Vertices[vFace.a];
	auto& VertexB = m_Mesh.m_Vertices[vFace.b];
	auto& VertexC = m_Mesh.m_Vertices[vFace.c];

	auto Box = __calcBoxInTextureCoord(VertexA.uv(), VertexB.uv(), VertexC.uv());

	for (int X = Box.first.x() * vResolution.x() - 1; X < Box.second.x() * vResolution.x() + 1; X++)
		for (int Y = Box.first.y() * vResolution.y() - 1; Y < Box.second.y() * vResolution.y() + 1; Y++)
			if (X >= 0 && X < vResolution.x() && Y >= 0 && Y < vResolution.y())
			{
				Eigen::Vector2f CenterUV = { (X + 0.5f) / vResolution.x(), (Y + 0.5f) / vResolution.y() };

				auto BarycentricCoord = __calcBarycentricCoord(VertexA.uv(), VertexB.uv(), VertexC.uv(), CenterUV);
				if ((BarycentricCoord.array() >= 0).all())
				{
					auto WorldPos = __calcTexelPosInWorld(VertexA.xyz(), VertexB.xyz(), VertexC.xyz(), BarycentricCoord);
					ResultSet.emplace_back(Eigen::Vector2i{ X, Y }, WorldPos, vFace);
				}
			}*/

	return ResultSet;
}

//*****************************************************************
//FUNCTION: 
std::vector<SCandidateInfo> CRayCastingBaker::executeIntersection(const STexelInfo& vInfo)
{
	auto RayOrigin = vInfo.TexelPosInWorld;
	auto RayDirection = __calcRayDirection(vInfo);

	std::vector<SCandidateInfo> Candidates;
	for (auto Index : __cullPointsByRay(RayOrigin, RayDirection))
	{
		auto& TestPoint = m_pCloud->points[Index];
		Eigen::Vector3f SurfelPos = TestPoint.getVector3fMap();
		Eigen::Vector3f SurfelNormal = TestPoint.getNormalVector3fMap();

		float DotNormal = SurfelNormal.dot(RayDirection);
		if (abs(DotNormal) < 1e-3f)
			continue;

		float Depth = SurfelNormal.dot(SurfelPos - RayOrigin) / DotNormal;
		auto HitPos = RayOrigin + Depth * RayDirection;
		
		float DistanceToCenter = (HitPos - SurfelPos).norm();
		//float DistanceToTexel = (HitPos - RayOrigin).norm();
		
		//固定半径
		if (DistanceToCenter <= SurfelRadius)
			Candidates.emplace_back(HitPos, Index);
	}

	return Candidates;
}

std::vector<SCandidateInfo> CRayCastingBaker::executeIntersection(const SSampleInfo& vInfo)
{

}

//*****************************************************************
//FUNCTION: 
Eigen::Vector3i CRayCastingBaker::calcTexelColor(const std::vector<SCandidateInfo>& vCandidates, const STexelInfo& vInfo)
{
	auto RayOrigin = vInfo.TexelPosInWorld;
	auto RayDirection = __calcRayDirection(vInfo);

	//找到最近的交点
	auto NearestSigenedDistance = std::numeric_limits<float>::max();
	for (const auto& [Intersection, _] : vCandidates)
	{
		float SigenedDistance = (Intersection - RayOrigin).dot(RayDirection);
		if (abs(NearestSigenedDistance) > abs(SigenedDistance))
			NearestSigenedDistance = SigenedDistance;
	}

	//交点剔除, 计算权重
	const auto Min = RayOrigin + (NearestSigenedDistance - DistanceThreshold) * RayDirection;
	const auto Max = RayOrigin + (NearestSigenedDistance + DistanceThreshold) * RayDirection;
	std::vector<std::pair<Eigen::Vector3i, float>> CulledCandidates;
	for (const auto& [Intersection, SurfelIndex] : vCandidates)
		if ((Intersection - Min).dot(Intersection - Max) <= 0)
		{
			auto& Point = m_pCloud->points[SurfelIndex];
			float Distance = (Intersection - Point.getVector3fMap()).norm() / SurfelRadius;
			float Weight = std::exp(-Distance * Distance * 20); //TODO: magic number

			CulledCandidates.emplace_back(Point.getRGBVector3i(), Weight);
		}
	
	//加权平均
	float SumWeight = 0.0f;
	Eigen::Vector3f WeightedColor = Eigen::Vector3f::Zero();
	for (auto& [Color, Weight] : CulledCandidates)
	{
		WeightedColor += Weight * Color.cast<float>();
		SumWeight += Weight;
	}

	return (WeightedColor / SumWeight).cast<int>();
}

//*****************************************************************
//FUNCTION: 
void CRayCastingBaker::__buildKdTree(PointCloud_t::Ptr vCloud)
{
	Eigen::Matrix<float, -1, -1, Eigen::RowMajor> PointsPos(vCloud->size(), 3);
	for (int i = 0; i < vCloud->size(); i++)
	{
		auto& Point = vCloud->points[i];
		Eigen::Vector3f PointPos{ Point.x, Point.y, Point.z };
		PointsPos.row(i) = PointPos;
	}

	flann::Matrix PointPos4Flann(PointsPos.data(), PointsPos.rows(), PointsPos.cols());
	auto pTree = new flann::Index<flann::L2<float>>(PointPos4Flann, flann::KDTreeIndexParams(4));
	pTree->buildIndex();
	m_KdTree = { pTree, std::move(PointsPos) };
}

//*****************************************************************
//FUNCTION: 
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

//*****************************************************************
//FUNCTION: 
Eigen::Vector3f CRayCastingBaker::__calcBarycentricCoord(const Eigen::Vector2f& vPointA, const Eigen::Vector2f& vPointB, const Eigen::Vector2f& vPointC, const Eigen::Vector2f& vPointP)
{
	auto A = vPointA - vPointC;
	auto B = vPointB - vPointC;
	auto C = vPointP - vPointC;

	float CoeffA = (C.x() * B.y() - C.y() * B.x()) / (A.x() * B.y() - A.y() * B.x());
	float CoeffB = (C.x() * A.y() - C.y() * A.x()) / (B.x() * A.y() - B.y() * A.x());
	float CoeffC = 1 - CoeffA - CoeffB;

	return { CoeffA, CoeffB, CoeffC };
}

//*****************************************************************
//FUNCTION: 
Eigen::Vector3f CRayCastingBaker::__calcTexelPosInWorld(const Eigen::Vector3f& vPosA, const Eigen::Vector3f& vPosB, const Eigen::Vector3f& vPosC, const Eigen::Vector3f& vBarycentricCoord)
{
	return vBarycentricCoord.x() * vPosA + vBarycentricCoord.y() * vPosB + vBarycentricCoord.z() * vPosC;
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
std::vector<pcl::index_t> CRayCastingBaker::__cullPointsByRay(const Eigen::Vector3f& vRayOrigin, const Eigen::Vector3f& vRayDirection)
{
	//暂用仅光线起点的半径搜索
	const float Radius = 100.0f;	//to config or calculate
	Eigen::Matrix<float, 1, 3, Eigen::RowMajor> SearchPos = vRayOrigin;
	flann::Matrix Query(SearchPos.data(), SearchPos.rows(), SearchPos.cols());
	std::vector<std::vector<pcl::index_t>> Indices;
	std::vector<std::vector<float>> Distances;

	m_KdTree.first->radiusSearch(Query, Indices, Distances, Radius, {});
	_ASSERTE(!Indices.empty());
	return Indices[0];
}