#include "pch.h"
#include "MeshPlaneIntersection.h"
#include <set>

using namespace hiveObliquePhotography::SceneReconstruction;

//*****************************************************************
//FUNCTION: 
void CMeshPlaneIntersection::execute(CMesh& vioMesh, const Eigen::Vector4f& vPlane)
{
	m_IntersectionPoints.clear();
	m_DissociatedPoints.clear();

	auto [BoxMin, BoxMax] = vioMesh.calcAABB();
	Eigen::Vector4f Plane = vPlane;
	if (__calcSignedDistance((BoxMin + BoxMax) / 2.0f, vPlane) < 0)
		Plane *= -1;
	
	std::set<SVertex> IntersectionPoints;
	std::set<int> Indices;
	for(auto FaceIter = vioMesh.m_Faces.begin(); FaceIter != vioMesh.m_Faces.end();)
	{
		std::array TempFace{ vioMesh.m_Vertices[FaceIter->a].xyz(), vioMesh.m_Vertices[FaceIter->b].xyz(), vioMesh.m_Vertices[FaceIter->c].xyz() };
		auto TempIntersectionSet = __calcIntersectionPoints(TempFace, Plane);
		bool bIsIntersected = false;
		if(!TempIntersectionSet.empty())
		{
			bIsIntersected = true;
			if (TempIntersectionSet.size() == 2)
			{
				auto FirstIter = TempIntersectionSet.begin();
				auto SecondIter = FirstIter + 1;
				IntersectionPoints.insert(SVertex{
					.x = (FirstIter->x + SecondIter->x) / 2,
					.y = (FirstIter->y + SecondIter->y) / 2,
					.z = (FirstIter->z + SecondIter->z) / 2,
				});
			}
			else
				IntersectionPoints.insert(TempIntersectionSet.begin(), TempIntersectionSet.end());
		}
		auto DissociatedSet = __tellDissociatedPoint(TempFace, Plane);
		if (bIsIntersected)
		{
			for (int i = 0; i < TempFace.size(); i++)
			{
				if (std::ranges::find(DissociatedSet, i) != DissociatedSet.end())
					Indices.insert(FaceIter->at(i));
			}
			FaceIter = vioMesh.m_Faces.erase(FaceIter);
		}
		else
		{
			if(DissociatedSet.empty())
				FaceIter = vioMesh.m_Faces.erase(FaceIter);
			else
				++FaceIter;
		}	
	}
	m_IntersectionPoints.assign(IntersectionPoints.begin(), IntersectionPoints.end());
	m_DissociatedPoints.assign(Indices.begin(), Indices.end());
}

//*****************************************************************
//FUNCTION: 
void CMeshPlaneIntersection::dumpIntersectionPoints(std::vector<SVertex>& vioIntersectionPoints)
{
	vioIntersectionPoints = m_IntersectionPoints;
}

//*****************************************************************
//FUNCTION: 
void CMeshPlaneIntersection::dumpDissociatedPoints(std::vector<int>& vioDissociatedPoints)
{
	vioDissociatedPoints = m_DissociatedPoints;
}

//*****************************************************************
//FUNCTION: intersections may repeat
std::vector<hiveObliquePhotography::SVertex> CMeshPlaneIntersection::__calcIntersectionPoints(const std::array<Eigen::Vector3f, 3>& vFace, const Eigen::Vector4f& vPlane)
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
std::vector<int> CMeshPlaneIntersection::__tellDissociatedPoint(const std::array<Eigen::Vector3f, 3>& vFace, const Eigen::Vector4f& vPlane)
{
	std::vector<int> DissociatedIndices;
	for(int i = 0; i < vFace.size(); i++)
		if (__calcSignedDistance(vFace[i], vPlane) < 0)
			DissociatedIndices.push_back(i);
	return DissociatedIndices;
}

//*****************************************************************
//FUNCTION: 
float CMeshPlaneIntersection::__calcSignedDistance(const Eigen::Vector3f& vPoint, const Eigen::Vector4f& vPlane) const
{
	return vPlane.dot(Eigen::Vector4f(vPoint.x(), vPoint.y(), vPoint.z(), 1.0f));
}
