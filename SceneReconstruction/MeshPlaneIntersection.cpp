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

	auto Box = vioMesh.calcAABB();
	Eigen::Vector3f Center = (Box.first + Box.second) / 2;
	Eigen::Vector3f PlaneNormal{ vPlane[0], vPlane[1],vPlane[2] };
	Eigen::Vector3f DefaultPlanePoint = __generateDefaultPlanePoint(vPlane);
	Eigen::Vector4f Plane;
	if ((DefaultPlanePoint - Center).dot(PlaneNormal) > 0)
		Plane = vPlane;
	else
		Plane = -vPlane;
	
	std::set<SVertex> IntersectionPoints;
	std::set<int> Indeices;
	std::vector<SFace>::iterator Iter = vioMesh.m_Faces.begin();
	for(;Iter!= vioMesh.m_Faces.end();)
	{
		std::vector<Eigen::Vector3f> TempFace{ vioMesh.m_Vertices[(*Iter).a].xyz(), vioMesh.m_Vertices[(*Iter).b].xyz(), vioMesh.m_Vertices[(*Iter).c].xyz() };
		auto TempIntersectionSet = __calcIntersectionPoints(TempFace, Plane);
		bool bIsIntersected = false;
		if(TempIntersectionSet.size())
		{
			bIsIntersected = true;
			IntersectionPoints.insert(TempIntersectionSet.begin(), TempIntersectionSet.end());
		}
		auto DissociatedSet = __tellDissociatedPoint(TempFace, Plane);
		if (bIsIntersected)
		{
			for (int i = 0; i < TempFace.size(); i++)
			{
				if (find(DissociatedSet.begin(), DissociatedSet.end(), i) != DissociatedSet.end())
					Indeices.insert((*Iter)[i]);
			}
			Iter = vioMesh.m_Faces.erase(Iter);
		}
		else
		{
			if(!DissociatedSet.size())
				Iter = vioMesh.m_Faces.erase(Iter);
			else
			    Iter++;
		}	
	}
	m_IntersectionPoints.assign(IntersectionPoints.begin(), IntersectionPoints.end());
	m_DissociatedPoints.assign(Indeices.begin(), Indeices.end());

	Eigen::Vector3f Direction;
	__findHightAxis(vioMesh, Direction);
	Direction = Direction.cross(PlaneNormal);
	__sortDissociatedIndices(vioMesh, m_DissociatedPoints, Direction);
	__sortIntersectionPoints(m_IntersectionPoints, Direction);

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


void CMeshPlaneIntersection::__sortDissociatedIndices(const CMesh& vMesh, std::vector<int>& vioDissociatedPoints, Eigen::Vector3f& vDirection)
{
	std::vector<int> OrderSet;
	auto compareV = [&](int vLhs, int vRhs) -> bool
	{
		return vMesh.m_Vertices[vLhs].xyz().dot(vDirection) < vMesh.m_Vertices[vRhs].xyz().dot(vDirection);
	};
	std::sort(vioDissociatedPoints.begin(), vioDissociatedPoints.end(), compareV);
	
	int CurrentIndex = vioDissociatedPoints[0];
	while (!vioDissociatedPoints.empty())
	{
		float MinDistance = FLT_MAX;
		std::vector<int>::iterator MinIt;
		for (auto Iter = vioDissociatedPoints.begin(); Iter != vioDissociatedPoints.end(); Iter++)
		{
			auto TempDis = (vMesh.m_Vertices[CurrentIndex].xyz() - vMesh.m_Vertices[(*Iter)].xyz()).norm();
			if (TempDis < MinDistance)
			{
				MinDistance = TempDis;
				MinIt = Iter;
			}
		}
		OrderSet.push_back(*MinIt);
		CurrentIndex = *MinIt;
		MinIt = vioDissociatedPoints.erase(MinIt);
	}
	vioDissociatedPoints.swap(OrderSet);
}

void CMeshPlaneIntersection::__sortIntersectionPoints(std::vector<SVertex>& vioIntersectionPoints, Eigen::Vector3f& vDirection)
{
	std::vector<SVertex> OrderSet;
	auto compareV = [&](const SVertex& vLhs, const SVertex& vRhs) -> bool
	{
		return vLhs.xyz().dot(vDirection) < vRhs.xyz().dot(vDirection);
	};
	std::sort(vioIntersectionPoints.begin(), vioIntersectionPoints.end(), compareV);

	SVertex CurrentVertex = vioIntersectionPoints[0];
	while(!vioIntersectionPoints.empty())
	{
		float MinDistance = FLT_MAX;
		std::vector<SVertex>::iterator MinIt;
		for(auto Iter = vioIntersectionPoints.begin(); Iter!=vioIntersectionPoints.end(); Iter++)
		{
			auto TempDis = (CurrentVertex.xyz() - (*Iter).xyz()).norm();
			if(TempDis < MinDistance)
			{
				MinDistance = TempDis;
				MinIt = Iter;
			}
		}
		OrderSet.push_back(*MinIt);
		CurrentVertex = *MinIt;
		MinIt = vioIntersectionPoints.erase(MinIt);
	}
	vioIntersectionPoints.swap(OrderSet);
}

void CMeshPlaneIntersection::__findHightAxis(const CMesh& vMesh, Eigen::Vector3f& voHightAxis)
{
	std::pair<int, int> UV;
	int Height;
	vMesh.calcModelPlaneAxis(UV, Height);
	for(int i = 0; i < 3; i++)
	{
		if (i == Height)
			voHightAxis[i] = 1.0f;
		else
			voHightAxis[i] = 0.0f;
	}
}