#include "pch.h"
#include "MeshPlaneIntersection.h"
#include<set>

using namespace hiveObliquePhotography::SceneReconstruction;

//*****************************************************************
//FUNCTION: 
void CMeshPlaneIntersection::execute(CMesh& vioMesh, const Eigen::Vector4f& vPlane)
{
	m_IntersectionPoints.clear();
	m_DissociatedPoints.clear();

	std::set<int> Indeices;
	std::vector<SFace>::iterator Iter = vioMesh.m_Faces.begin();
	for(;Iter!= vioMesh.m_Faces.end();)
	{
		std::vector<Eigen::Vector3f> TempFace{ vioMesh.m_Vertices[(*Iter).a].xyz(), vioMesh.m_Vertices[(*Iter).b].xyz(), vioMesh.m_Vertices[(*Iter).c].xyz() };
		auto TempIntersectionSet = __calcIntersectionPoints(TempFace, vPlane);
		bool bIsIntersected = false;
		if(TempIntersectionSet.size())
		{
			bIsIntersected = true;
			m_IntersectionPoints.insert(m_IntersectionPoints.end(), TempIntersectionSet.begin(), TempIntersectionSet.end());
		}
		if (bIsIntersected)
		{
			auto DissociatedSet = __tellDissociatedPoint(TempFace, vPlane);
			for (int i = 0; i < TempFace.size(); i++)
			{
				if (find(DissociatedSet.begin(), DissociatedSet.end(), i) != DissociatedSet.end())
					Indeices.insert((*Iter)[i]);
			}
			Iter = vioMesh.m_Faces.erase(Iter);
		}
		else
			Iter++;
	}
	m_DissociatedPoints.assign(Indeices.begin(), Indeices.end());
	return;
}

//*****************************************************************
//FUNCTION: 
void CMeshPlaneIntersection::dumpIntersectionPoints(std::vector<SVertex>& vioIntersectionPoints)
{
	vioIntersectionPoints = m_IntersectionPoints;
	return;
}

//*****************************************************************
//FUNCTION: 
void CMeshPlaneIntersection::dumpDissociatedPoints(std::vector<int>& vioDissociatedPoints)
{
	vioDissociatedPoints = m_DissociatedPoints;
	return;
}

//*****************************************************************
//FUNCTION: 
std::vector<hiveObliquePhotography::SVertex> CMeshPlaneIntersection::__calcIntersectionPoints(const std::vector<Eigen::Vector3f>& vFace, const Eigen::Vector4f& vPlane)
{
	std::vector<SVertex> HitPointSet;
	Eigen::Vector3f PlaneNormal{ vPlane[0], vPlane[1],vPlane[2] };
	Eigen::Vector3f DefaultPlanePoint = __generateDefaultPlanePoint(vPlane);
	for(int i = 0; i < vFace.size(); i++)
	{
		Eigen::Vector3f Origin = vFace[i];
		Eigen::Vector3f Direction = vFace[(i + 1) % 3] - vFace[i];
		if (Direction.dot(PlaneNormal) < 0)
			Direction = - Direction;
		
		float DotNormal = PlaneNormal.dot(Direction);
		if (DotNormal < 1e-3f)
			continue;

		float Depth = PlaneNormal.dot(DefaultPlanePoint - Origin) / DotNormal;
		Eigen::Vector3f HitPos = Origin + Depth * Direction;

		if((HitPos - vFace[i]).dot(HitPos - vFace[(i + 1) % 3]) < 0 || HitPos == vFace[i])
		{
			SVertex TempVertex; TempVertex.x = HitPos[0]; TempVertex.y = HitPos[1]; TempVertex.z = HitPos[2];
			HitPointSet.push_back(TempVertex);
		}	
	}
	return HitPointSet;
}

//*****************************************************************
//FUNCTION: 
std::vector<int> CMeshPlaneIntersection::__tellDissociatedPoint(const std::vector<Eigen::Vector3f>& vFace, const Eigen::Vector4f& vPlane)
{
	Eigen::Vector3f DefaultPlanePoint = __generateDefaultPlanePoint(vPlane);
	std::vector<int> DissociatedIndices;
	Eigen::Vector3f PlaneNormal{ vPlane[0], vPlane[1],vPlane[2] };
	for(int i = 0; i < vFace.size(); i++)
	{
		auto Vector = DefaultPlanePoint - vFace[i];
		if (Vector.dot(PlaneNormal) > 0)
			DissociatedIndices.push_back(i);
	}
	return DissociatedIndices;
}

Eigen::Vector3f CMeshPlaneIntersection::__generateDefaultPlanePoint(const Eigen::Vector4f& vPlane)
{
	int i = 0;
	for (; i < 3; i++)
	{
		if (vPlane[i])
		  break;
	}
	Eigen::Vector3f DefaultPlanePoint;
	for(int k = 0; k < 3; k++)
	{
		if (k == i)
			DefaultPlanePoint[k] = -vPlane[3] / vPlane[i];
		else
			DefaultPlanePoint[k] = 0.0f;
	}
	return DefaultPlanePoint;
}