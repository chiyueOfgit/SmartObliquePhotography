#include "pch.h"
#include "RayCastingBaker.h"
#include <common/MathInterface.h>

using namespace hiveObliquePhotography::SceneReconstruction;

_REGISTER_NORMAL_PRODUCT(CRayCastingBaker, KEYWORD::RAYCASTING_TEXTUREBAKER)

//*****************************************************************
//FUNCTION: 
hiveObliquePhotography::CImage<std::array<int, 3>> CRayCastingBaker::bakeTexture(PointCloud_t::Ptr vPointCloud, const Eigen::Vector2i& vResolution)
{
	m_pCloud = vPointCloud;
	__buildKdTree(m_pCloud);

	m_SurfelRadius = m_pConfig->getAttribute<float>(KEYWORD::SURFEL_RADIUS).value();
	m_NumSample = m_pConfig->getAttribute<int>(KEYWORD::NUM_SAMPLE).value();
	m_SearchRadius = m_pConfig->getAttribute<float>(KEYWORD::SEARCH_RADIUS).value();
	m_DistanceThreshold = m_pConfig->getAttribute<float>(KEYWORD::DISTANCE_THRESHOLD).value();

	auto Res = m_pConfig->getAttribute<std::tuple<int, int>>(KEYWORD::RESOLUTION).value();
	Eigen::Vector2i Resolution = { std::get<0>(Res), std::get<1>(Res) };

	Eigen::Matrix<std::array<int, 3>, -1, -1> Texture(Resolution.y(), Resolution.x());
	for (const auto& Face : m_Mesh.m_Faces)
		for (const auto& PerTexel : findSamplesPerFace(Face, Resolution))
		{
			std::vector<std::array<int, 3>> TexelColorSet;
			TexelColorSet.reserve(PerTexel.RaySet.size());
			for (const auto& Ray : PerTexel.RaySet)
			{
				auto Candidates = executeIntersection(Ray);
				if (Candidates.empty())
					continue;
				TexelColorSet.push_back(calcTexelColor(Candidates, Ray));
			}
			if (TexelColorSet.empty())
				continue;
			Texture(PerTexel.TexelCoord.y(), PerTexel.TexelCoord.x()) = __mixSamplesColor(TexelColorSet);
		}
	
	CImage<std::array<int, 3>> ResultTexture;
	ResultTexture.fillColor(Resolution.y(), Resolution.x(), Texture.data());
	return ResultTexture;
}

//*****************************************************************
//FUNCTION: 
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
				/*std::vector<Eigen::Vector3f> SampleSet;
				SampleSet.reserve(m_NumSample);

				auto BarycentricCoord = hiveMath::hiveGenerateRandomRealSet(0.0f, (float)vResolution.x(), m_NumSample * 3);
				for (int m = 0; m < m_NumSample; m++)
				{
					auto Sum = BarycentricCoord[m * 3] + BarycentricCoord[m * 3 + 1] + BarycentricCoord[m * 3 + 2];
					if (Sum > 0)
						SampleSet.emplace_back(BarycentricCoord[m * 3] / Sum, BarycentricCoord[m * 3 + 1] / Sum, BarycentricCoord[m * 3 + 2] / Sum);
				}

				std::vector<SRay> RaySet;
				RaySet.reserve(m_NumSample);

				for (const auto& Sample : SampleSet)
					RaySet.push_back(__calcRay(vFace, Sample));

				if (!RaySet.empty())
					ResultSet.emplace_back(Eigen::Vector2i{ i, k }, RaySet);*/

				std::vector<Eigen::Vector2f> SampleSet;
				SampleSet.reserve(m_NumSample);
				SampleSet.emplace_back((i + 0.5f) / vResolution.x(), (k + 0.5f) / vResolution.y());

				auto USampleSet = hiveMath::hiveGenerateRandomRealSet((i + 0.0f) / vResolution.x(), (i + 1.0f) / vResolution.x(), m_NumSample - 1);
				auto VSampleSet = hiveMath::hiveGenerateRandomRealSet((k + 0.0f) / vResolution.y(), (k + 1.0f) / vResolution.y(), m_NumSample - 1);
				for (int m = 0; m < m_NumSample - 1; ++m)
					SampleSet.emplace_back(USampleSet[m], VSampleSet[m]);

				std::vector<SRay> RaySet;
				RaySet.reserve(m_NumSample);
				for (const auto& Sample : SampleSet)
				{
					auto BarycentricCoord = __calcBarycentricCoord(VertexA.uv(), VertexB.uv(), VertexC.uv(), Sample);
					if ((BarycentricCoord.array() >= 0).all())
						RaySet.push_back(__calcRay(vFace, BarycentricCoord));
				}
				if (!RaySet.empty())
					ResultSet.emplace_back(Eigen::Vector2i{ i, k }, RaySet);
			}

	return ResultSet;
}

//*****************************************************************
//FUNCTION: 
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
		if (DistanceToCenter <= m_SurfelRadius)
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
	const auto Min = RayOrigin + (NearestSigenedDistance - m_DistanceThreshold) * RayDirection;
	const auto Max = RayOrigin + (NearestSigenedDistance + m_DistanceThreshold) * RayDirection;
	std::vector<std::pair<Eigen::Vector3i, float>> CulledCandidates;
	for (const auto& [Intersection, SurfelIndex] : vCandidates)
		if ((Intersection - Min).dot(Intersection - Max) <= 0)
		{
			auto& Point = m_pCloud->points[SurfelIndex];
			float Distance = (Intersection - Point.getVector3fMap()).norm() / m_SurfelRadius;
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
	const float Radius = m_SearchRadius;	//to config or calculate
	Eigen::Matrix<float, 1, 3, Eigen::RowMajor> SearchPos = vRayOrigin;
	flann::Matrix Query(SearchPos.data(), SearchPos.rows(), SearchPos.cols());
	std::vector<std::vector<pcl::index_t>> Indices;
	std::vector<std::vector<float>> Distances;

	m_KdTree.first->radiusSearch(Query, Indices, Distances, Radius, {});
	_ASSERTE(!Indices.empty());
	return Indices[0];
}

//*****************************************************************
//FUNCTION: 
std::array<int, 3> CRayCastingBaker::__mixSamplesColor(const std::vector<std::array<int, 3>>& vColorSet) const
{
	//k-means?
	std::array AverageColor = { 0 ,0 ,0 };
	if (!vColorSet.empty())
	{
		for (const auto& [R, G, B] : vColorSet)
		{
			AverageColor[0] += R;
			AverageColor[1] += G;
			AverageColor[2] += B;
		}
		for (auto& i : AverageColor)
			i /= vColorSet.size();
	}

	return AverageColor;
}
