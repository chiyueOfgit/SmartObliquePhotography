#include "pch.h"
#include "MeshPlaneIntersection.h"

using namespace hiveObliquePhotography::SceneReconstruction;

//*****************************************************************
//FUNCTION: 
void CMeshPlaneIntersection::execute(CMesh& vioMesh, const Eigen::Vector4f& vPlane)
{
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
			//TODO: 新生成点属性；
		}
		if (bIsIntersected)
		{
			auto DissociatedSet = __tellDissociatedPoint(TempFace, vPlane);
			for (int i = 0; i < TempFace.size(); i++)
			{
				if (find(DissociatedSet.begin(), DissociatedSet.end(), (*Iter)[i]) != DissociatedSet.end())
					m_DissociatedPoints.push_back((*Iter)[i]);
			}
			Iter = vioMesh.m_Faces.erase(Iter);
		}
		else
			Iter++;
	}

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
	Eigen::Vector3f DefaultPlanePoint{ 0.0f, 0.0f, -vPlane[3] / vPlane[2] };
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
		auto HitPos = Origin + Depth * Direction;

		if((HitPos - vFace[i]).dot(HitPos - vFace[(i + 1) % 3]) < 0)
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
	Eigen::Vector3f DefaultPlanePoint{ 0.0f, 0.0f, -vPlane[3] / vPlane[2] };
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