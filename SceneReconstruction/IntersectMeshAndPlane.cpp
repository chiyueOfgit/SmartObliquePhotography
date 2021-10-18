#include "pch.h"
#include "IntersectMeshAndPlane.h"
#include <set>

using hiveObliquePhotography::SVertex;

float __calcSignedDistance(const Eigen::Vector3f& vPoint, const Eigen::Vector4f& vPlane);
std::vector<SVertex> __calcIntersectionPoints(const std::array<Eigen::Vector3f, 3>& vFace, const Eigen::Vector4f& vPlane);
std::vector<int> __tellDissociatedPoint(const std::array<Eigen::Vector3f, 3>& vFace, const Eigen::Vector4f& vPlane);

//*****************************************************************
//FUNCTION: 
void hiveObliquePhotography::SceneReconstruction::intersectMeshAndPlane
	(CMesh& vioMesh, const Eigen::Vector4f& vPlane, std::vector<SVertex>& voIntersectionPoints, std::vector<int>& voDissociatedIndices)
{
	auto [BoxMin, BoxMax] = vioMesh.calcAABB();
	Eigen::Vector4f Plane = vPlane;
	if (__calcSignedDistance((BoxMin + BoxMax) / 2.0f, vPlane) > 0)
		Plane *= -1;

	std::set<SVertex> IntersectionPoints;
	std::set<int> DissociatedIndices;
	for (auto FaceIter = vioMesh.m_Faces.begin(); FaceIter != vioMesh.m_Faces.end();)
	{
		std::array TempFace{ vioMesh.m_Vertices[FaceIter->a].xyz(), vioMesh.m_Vertices[FaceIter->b].xyz(), vioMesh.m_Vertices[FaceIter->c].xyz() };
		auto IntersectionsOfFace = __calcIntersectionPoints(TempFace, Plane);
		auto DissociatedIndicesOfFace = __tellDissociatedPoint(TempFace, Plane);

		const auto NumIntersection = IntersectionsOfFace.size();
		if (NumIntersection == 0)
		{
			if (DissociatedIndicesOfFace.empty())
				FaceIter = vioMesh.m_Faces.erase(FaceIter);
			else
				++FaceIter;
		}
		else
		{
			for (size_t i = 0; i < TempFace.size(); ++i)
				if (std::ranges::find(DissociatedIndicesOfFace, i) != DissociatedIndicesOfFace.end())
					DissociatedIndices.insert(FaceIter->at(i));
			FaceIter = vioMesh.m_Faces.erase(FaceIter);

			if (NumIntersection == 1)
				IntersectionPoints.insert(IntersectionsOfFace.front());
			else// if (NumIntersection == 2)
			{
				auto FirstIter = IntersectionsOfFace.begin();
				auto SecondIter = FirstIter + 1;
				IntersectionPoints.insert(SVertex{
					.x = (FirstIter->x + SecondIter->x) / 2,
					.y = (FirstIter->y + SecondIter->y) / 2,
					.z = (FirstIter->z + SecondIter->z) / 2,
					});
			}
		}
	}
	voIntersectionPoints.assign(IntersectionPoints.begin(), IntersectionPoints.end());
	voDissociatedIndices.assign(DissociatedIndices.begin(), DissociatedIndices.end());
}

//*****************************************************************
//FUNCTION: 
float __calcSignedDistance(const Eigen::Vector3f& vPoint, const Eigen::Vector4f& vPlane)
{
	return vPlane.dot(Eigen::Vector4f(vPoint.x(), vPoint.y(), vPoint.z(), 1.0f));
}

//*****************************************************************
//FUNCTION: intersections may repeat
std::vector<SVertex> __calcIntersectionPoints(const std::array<Eigen::Vector3f, 3>& vFace, const Eigen::Vector4f& vPlane)
{
	std::vector<SVertex> HitPointSet;
	for(size_t i = 0; i < vFace.size(); ++i)
	{
		const Eigen::Vector3f& OriginVertex = vFace[i];
		const Eigen::Vector3f& NextVertex = vFace[(i + 1) % 3];
		
		Eigen::Vector3f Direction = NextVertex - OriginVertex;
		const float DotNormal = Direction.dot(Eigen::Vector3f(vPlane[0], vPlane[1], vPlane[2]));
		const float OriginPlaneDistance = __calcSignedDistance(OriginVertex, vPlane);

		if (abs(DotNormal) < 1e-3f)
			continue;
		
		Eigen::Vector3f Intersection = OriginVertex - OriginPlaneDistance / DotNormal * Direction;
		if ((Intersection - OriginVertex).dot(Intersection - NextVertex) <= 0)
			HitPointSet.push_back(SVertex{
				.x = Intersection[0],
				.y = Intersection[1],
				.z = Intersection[2],
				});
	}
	return HitPointSet;
}

//*****************************************************************
//FUNCTION: 
std::vector<int> __tellDissociatedPoint(const std::array<Eigen::Vector3f, 3>& vFace, const Eigen::Vector4f& vPlane)
{
	std::vector<int> DissociatedIndices;
	for(size_t i = 0; i < vFace.size(); ++i)
		if (__calcSignedDistance(vFace[i], vPlane) < 0)
			DissociatedIndices.push_back(i);
	return DissociatedIndices;
}
