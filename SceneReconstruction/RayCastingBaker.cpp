#include "pch.h"
#include "RayCastingBaker.h"
#include <common/MathInterface.h>

using namespace hiveObliquePhotography::SceneReconstruction;

_REGISTER_NORMAL_PRODUCT(CRayCastingBaker, KEYWORD::RAYCASTING_TEXTUREBAKER)

//TODO: magic number
constexpr float SurfelRadius = 5.0f;
constexpr float SearchRadius = 100.0f;
constexpr float DistanceThreshold = 0.2f;
constexpr float SampleInterval = 0.01f;

//*****************************************************************
//FUNCTION: 
hiveObliquePhotography::CImage<std::array<int, 3>> CRayCastingBaker::bakeTexture(PointCloud_t::Ptr vPointCloud, const Eigen::Vector2i& vResolution)
{
	m_pCloud = vPointCloud;
	__buildKdTree(m_pCloud);

	Eigen::Matrix<std::array<int, 3>, -1, -1> Texture(vResolution.y(), vResolution.x());
	for (int i = 0; i < Texture.rows(); i++)
		for (int k = 0; k < Texture.cols(); k++)
			Texture(i, k) = { 255, 255, 255 };
	for (const auto& Face : m_Mesh.m_Faces)
		for (const auto& PerTexel : findSamplesPerFace(Face, vResolution))
		{
			std::vector<std::array<int, 3>> TexelColorSet;
			TexelColorSet.reserve(PerTexel.RaySet.size());
			for (const auto& Ray : PerTexel.RaySet)
			{
				auto Candidates = executeIntersection(Ray);
				TexelColorSet.push_back(calcTexelColor(Candidates, Ray));
			}

			std::array AverageColor = { 0 ,0 ,0 };
			for (const auto& [R, G, B] : TexelColorSet)
			{
				AverageColor[0] += R;
				AverageColor[1] += G;
				AverageColor[2] += B;
			}
			if (!TexelColorSet.empty())
			{
				for (auto& i : AverageColor)
					i /= TexelColorSet.size();
				Texture(PerTexel.TexelCoord.y(), PerTexel.TexelCoord.x()) = AverageColor;
			}

		}
	
	CImage<std::array<int, 3>> ResultTexture;
	ResultTexture.fillColor(vResolution.y(), vResolution.x(), Texture.data());
	return ResultTexture;
}

std::vector<STexelInfo> CRayCastingBaker::findSamplesPerFace(const SFace& vFace, const Eigen::Vector2i& vResolution) const
{
	const auto& VertexA = m_Mesh.m_Vertices[vFace.a];
	const auto& VertexB = m_Mesh.m_Vertices[vFace.b];
	const auto& VertexC = m_Mesh.m_Vertices[vFace.c];
	
	std::vector<STexelInfo> ResultSet;
	auto [Min, Max] = __calcBoxInTextureCoord(VertexA.uv(), VertexB.uv(), VertexC.uv());
	const Eigen::Vector2i FromCoord = { Min.x() * vResolution.x() - 1, Min.y() * vResolution.y() - 1 };
	const Eigen::Vector2i ToCoord = { Max.x() * vResolution.x() + 1, Max.y() * vResolution.y() + 1 };
	for (auto i = FromCoord.x(); i < ToCoord.x(); ++i)
		for (auto k = FromCoord.y(); k < ToCoord.y(); ++k)
			if (i >= 0 && i < vResolution.x() && k >= 0 && k < vResolution.y())
			{
				STexelInfo TexelSampleInfo{ .TexelCoord= { i, k }, .RaySet = {} };
				//TexelSampleInfo.SamplePosSetInWorld.reserve(间隔0.2对应的数目);
				std::pair FromSample = { (i + 0.0f) / vResolution.x(), (k + 0.0f) / vResolution.y() };
				std::pair ToSample = { (i + 1.0f) / vResolution.x(), (k + 1.0f) / vResolution.y() };
				const int NumRandomSample = 4;
				auto SampleU = hiveMath::hiveGenerateRandomRealSet<float>(FromSample.first, ToSample.first, NumRandomSample);
				auto SampleV = hiveMath::hiveGenerateRandomRealSet<float>(FromSample.second, ToSample.second, NumRandomSample);
				for (int i = 0; i < NumRandomSample; i++)
				{
					auto u = SampleU[i];
					auto v = SampleV[i];
					//auto u = (i + 0.5f) / vResolution.x();
					//auto v = (k + 0.5f) / vResolution.y();
					auto BarycentricCoord = __calcBarycentricCoord(VertexA.uv(), VertexB.uv(), VertexC.uv(), {u, v});
					if ((BarycentricCoord.array() >= 0).all())
						TexelSampleInfo.RaySet.push_back(__calcRay(vFace, BarycentricCoord));
				}
				if (!TexelSampleInfo.RaySet.empty())
					ResultSet.push_back(std::move(TexelSampleInfo));
			}

	return ResultSet;
}

std::vector<SCandidateInfo> CRayCastingBaker::executeIntersection(const SRay& vRay) const
{
	const auto RayOrigin = vRay.Origin;
	const auto RayDirection = vRay.Direction;

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

//*****************************************************************
//FUNCTION: 
std::array<int, 3> CRayCastingBaker::calcTexelColor(const std::vector<SCandidateInfo>& vCandidates, const SRay& vRay) const
{
	const auto RayOrigin = vRay.Origin;
	const auto RayDirection = vRay.Direction;

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

	Eigen::Vector3i Color = (WeightedColor / SumWeight).cast<int>();
	
	//if ((Color.x() || Color.y() || Color.z()) == 0)
	//{
	//	std::string Info;
	//	Info += _FORMAT_STR3("\nWeihgtedColor: %1% %2% %3%", std::to_string(WeightedColor.x()), std::to_string(WeightedColor.y()), std::to_string(WeightedColor.z()));
	//	Info += _FORMAT_STR1("\nSumWewight: %1%\n", std::to_string(SumWeight));
	//	hiveEventLogger::hiveOutputEvent(Info);
	//}

	return { Color.x(), Color.y(), Color.z() };
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
std::pair<Eigen::Vector2f, Eigen::Vector2f> CRayCastingBaker::__calcBoxInTextureCoord(const Eigen::Vector2f& vPointA, const Eigen::Vector2f& vPointB, const Eigen::Vector2f& vPointC) const
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
Eigen::Vector3f CRayCastingBaker::__calcBarycentricCoord(const Eigen::Vector2f& vPointA, const Eigen::Vector2f& vPointB, const Eigen::Vector2f& vPointC, const Eigen::Vector2f& vPointP) const
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
SRay CRayCastingBaker::__calcRay(const SFace& vFace, const Eigen::Vector3f& vBarycentricCoord) const
{
	auto PosA = m_Mesh.m_Vertices[vFace.a].xyz();
	auto PosB = m_Mesh.m_Vertices[vFace.b].xyz();
	auto PosC = m_Mesh.m_Vertices[vFace.c].xyz();

	Eigen::Vector3f RayOrigin = vBarycentricCoord.x() * PosA + vBarycentricCoord.y() * PosB + vBarycentricCoord.z() * PosC;
	Eigen::Vector3f RayDirection = (PosB - PosA).cross(PosC - PosA).normalized();
	
	return { RayOrigin, RayDirection };
}

//*****************************************************************
//FUNCTION: 
std::vector<pcl::index_t> CRayCastingBaker::__cullPointsByRay(const Eigen::Vector3f& vRayOrigin, const Eigen::Vector3f& vRayDirection) const
{
	//暂用仅光线起点的半径搜索
	const float Radius = SearchRadius;	//to config or calculate
	Eigen::Matrix<float, 1, 3, Eigen::RowMajor> SearchPos = vRayOrigin;
	flann::Matrix Query(SearchPos.data(), SearchPos.rows(), SearchPos.cols());
	std::vector<std::vector<pcl::index_t>> Indices;
	std::vector<std::vector<float>> Distances;

	m_KdTree.first->radiusSearch(Query, Indices, Distances, Radius, {});
	_ASSERTE(!Indices.empty());
	return Indices[0];
}