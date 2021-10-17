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
		std::vector<Eigen::Vector3f> TempFace{ vioMesh.m_Vertices[FaceIter->a].xyz(), vioMesh.m_Vertices[FaceIter->b].xyz(), vioMesh.m_Vertices[FaceIter->c].xyz() };
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
//FUNCTION: 
std::vector<hiveObliquePhotography::SVertex> CMeshPlaneIntersection::__calcIntersectionPoints(const std::vector<Eigen::Vector3f>& vFace, const Eigen::Vector4f& vPlane)
{
	std::vector<SVertex> HitPointSet;
	Eigen::Vector3f PlaneNormal{ vPlane[0], vPlane[1],vPlane[2] };
	for(int i = 0; i < vFace.size(); i++)
	{
		Eigen::Vector3f Origin = vFace[i];
		Eigen::Vector3f Direction = vFace[(i + 1) % 3] - vFace[i];
		float DotNormal = PlaneNormal.dot(Direction);
		if (DotNormal < 0)
		{
			DotNormal *= -1;
			Direction *= -1;
		}

		if(abs(__calcSignedDistance(Origin, vPlane)) < 1e-3f && abs(__calcSignedDistance(vFace[(i + 1) % 3], vPlane)) < 1e-3f)
		{
			HitPointSet.push_back(SVertex{
				.x = Origin[0],
				.y = Origin[1],
				.z = Origin[2],
			});
			HitPointSet.push_back(SVertex{
				.x = vFace[(i + 1) % 3][0],
				.y = vFace[(i + 1) % 3][1],
				.z = vFace[(i + 1) % 3][2],
			});
		}
		else
		{
			float Depth = -__calcSignedDistance(Origin, vPlane) / DotNormal;
			Eigen::Vector3f HitPos = Origin + Depth * Direction;

			if ((HitPos - vFace[i]).dot(HitPos - vFace[(i + 1) % 3]) < 0 || HitPos == vFace[i])
			{
				HitPointSet.push_back(SVertex{
					.x = HitPos[0],
					.y = HitPos[1],
					.z = HitPos[2],
				});
			}
		}
	}
	return HitPointSet;
}

//*****************************************************************
//FUNCTION: 
std::vector<int> CMeshPlaneIntersection::__tellDissociatedPoint(const std::vector<Eigen::Vector3f>& vFace, const Eigen::Vector4f& vPlane)
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
