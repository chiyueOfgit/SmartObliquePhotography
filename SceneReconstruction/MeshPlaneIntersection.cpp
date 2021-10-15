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
			if (TempIntersectionSet.size() == 2)
			{
				SVertex AverageVertex;
				auto FirstIter = TempIntersectionSet.begin();
				auto SecondIter = FirstIter + 1;
				AverageVertex.x = (FirstIter->x + SecondIter->x) / 2;
				AverageVertex.y = (FirstIter->y + SecondIter->y) / 2;
				AverageVertex.z = (FirstIter->z + SecondIter->z) / 2;
				IntersectionPoints.insert(AverageVertex);
			}
			else
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
	__findHeightAxis(vioMesh, Direction);
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

		if(abs(PlaneNormal.dot(DefaultPlanePoint - Origin)) < 1e-3f && abs(PlaneNormal.dot(DefaultPlanePoint - vFace[(i + 1) % 3])) < 1e-3f)
		{
			SVertex TempVertex; TempVertex.x = Origin[0]; TempVertex.y = Origin[1]; TempVertex.z = Origin[2];
			HitPointSet.push_back(TempVertex);
			SVertex OtherTempVertex; OtherTempVertex.x = vFace[(i + 1) % 3][0]; OtherTempVertex.y = vFace[(i + 1) % 3][1]; OtherTempVertex.z = vFace[(i + 1) % 3][2];
			HitPointSet.push_back(OtherTempVertex);
		}
		else
		{
			float Depth = PlaneNormal.dot(DefaultPlanePoint - Origin) / DotNormal;
			Eigen::Vector3f HitPos = Origin + Depth * Direction;

			if ((HitPos - vFace[i]).dot(HitPos - vFace[(i + 1) % 3]) < 0 || HitPos == vFace[i])
			{
				SVertex TempVertex; TempVertex.x = HitPos[0]; TempVertex.y = HitPos[1]; TempVertex.z = HitPos[2];
				HitPointSet.push_back(TempVertex);
			}
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

	std::vector<SVertex> VertexSet;
	for(auto Index: vioDissociatedPoints)
		VertexSet.push_back(vMesh.m_Vertices[Index]);
	__sortByVertexLoop<int>(vioDissociatedPoints, VertexSet);
}

void CMeshPlaneIntersection::__sortIntersectionPoints(std::vector<SVertex>& vioIntersectionPoints, Eigen::Vector3f& vDirection)
{
	std::vector<SVertex> OrderSet;
	auto compareV = [&](const SVertex& vLhs, const SVertex& vRhs) -> bool
	{
		return vLhs.xyz().dot(vDirection) < vRhs.xyz().dot(vDirection);
	};
	std::sort(vioIntersectionPoints.begin(), vioIntersectionPoints.end(), compareV);
	__sortByVertexLoop<SVertex>(vioIntersectionPoints, vioIntersectionPoints);
}

template <typename Type>
void CMeshPlaneIntersection::__sortByVertexLoop(std::vector<Type>& vioOriginSet, std::vector<SVertex>& vVertexSet)
{
	if (vVertexSet.empty())
		return;
	std::vector<int> Indices;
	std::vector<Type> OrderSet;
	
	SVertex CurrentVertex = vVertexSet[0];
	std::vector<bool> Flag(vVertexSet.size(), false);
	auto Count = vVertexSet.size();
	while (Count > 0)
	{
		float MinDistance = FLT_MAX;
		int MinIndex;
		for (int i = 0; i < vVertexSet.size();i++)
		{
			if (!Flag[i])
			{
				auto TempDis = (CurrentVertex.xyz() - vVertexSet[i].xyz()).norm();
				if (TempDis < MinDistance)
				{
					MinDistance = TempDis;
					MinIndex = i;
				}
			}
		}
		Indices.push_back(MinIndex);
		CurrentVertex = vVertexSet[MinIndex];
		Flag[MinIndex] = true;
		Count--;
	}
	for (auto Index : Indices)
		OrderSet.push_back(vioOriginSet[Index]);
	vioOriginSet.swap(OrderSet);
}

void CMeshPlaneIntersection::__findHeightAxis(const CMesh& vMesh, Eigen::Vector3f& voHeightAxis)
{
	std::pair<int, int> UV;
	int Height;
	vMesh.calcModelPlaneAxis(UV, Height);
	for(int i = 0; i < 3; i++)
	{
		if (i == Height)
			voHeightAxis[i] = 1.0f;
		else
			voHeightAxis[i] = 0.0f;
	}
}