#include "pch.h"
#include "BasicMeshSuture.h"
#include <vcg/complex/algorithms/clean.h>
#include "MeshPlaneIntersection.h"
#include "FindSplitPlane.h"
#include "VcgMesh.hpp"

using namespace hiveObliquePhotography::SceneReconstruction;

_REGISTER_NORMAL_PRODUCT(CBasicMeshSuture, KEYWORD::BASIC_MESH_SUTURE)

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::sutureMeshes()
{
	//TODO: nullptr
	pcl::PointCloud<pcl::PointXYZ>::Ptr CloudOne, CloudTwo;
	auto SegmentPlane = __calcSegmentPlane(CloudOne, CloudTwo);
	
	std::vector<int> LHSDissociatedIndices, RHSDissociatedIndices;
	std::vector<SVertex> LHSIntersectionPoints, RHSIntersectionPoints, PublicVertices;
	__executeIntersection(m_MeshLHS, SegmentPlane, LHSDissociatedIndices, LHSIntersectionPoints);
	__executeIntersection(m_MeshRHS, SegmentPlane, RHSDissociatedIndices, RHSIntersectionPoints);

	__generatePublicVertices(LHSIntersectionPoints, RHSIntersectionPoints, PublicVertices);
	__connectVerticesWithMesh(m_MeshLHS, LHSDissociatedIndices, PublicVertices);
	__connectVerticesWithMesh(m_MeshRHS, RHSDissociatedIndices, PublicVertices);

	__removeUnreferencedVertex(m_MeshLHS);
	__removeUnreferencedVertex(m_MeshRHS);
}

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::dumpMeshes(CMesh& voLHSMesh, CMesh& voRHSMesh)
{
	voLHSMesh = m_MeshLHS;
	voRHSMesh = m_MeshRHS;
}

//*****************************************************************
//FUNCTION: 
Eigen::Vector4f CBasicMeshSuture::__calcSegmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr vCloudOne, pcl::PointCloud<pcl::PointXYZ>::Ptr vCloudTwo)
{
	return CFindSplitPlane().execute(vCloudOne, vCloudTwo);
}

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::__executeIntersection(CMesh& vioMesh, const Eigen::Vector4f& vPlane, std::vector<int>& voDissociatedIndices, std::vector<SVertex>& voIntersectionPoints)
{
	CMeshPlaneIntersection MeshPlaneIntersection;
	MeshPlaneIntersection.execute(vioMesh, vPlane);
	MeshPlaneIntersection.dumpDissociatedPoints(voDissociatedIndices);
	MeshPlaneIntersection.dumpIntersectionPoints(voIntersectionPoints);
}

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::__generatePublicVertices(const std::vector<SVertex>& vLHSIntersectionPoints, const std::vector<SVertex>& vRHSIntersectionPoints, std::vector<SVertex>& voPublicVertices)
{
	if (voPublicVertices.size() > 0)
		voPublicVertices.clear();
	std::vector<SVertex> MajorPoints, MinorPoints, PairedMinorPoints;
	std::map<SVertex, SVertex> PairingPoints, PairingPointsAmended;
	bool Comparation = vLHSIntersectionPoints.size() > vRHSIntersectionPoints.size();
	MajorPoints = Comparation ? vLHSIntersectionPoints : vRHSIntersectionPoints;
	MinorPoints = Comparation ? vRHSIntersectionPoints : vLHSIntersectionPoints;

	for (auto MajorPoint : MajorPoints)
	{
		SVertex NearestPoint = __findNearestPoint(MinorPoints, MajorPoint);
		PairingPoints.insert(std::pair<SVertex, SVertex>(MajorPoint, NearestPoint));
		PairedMinorPoints.push_back(NearestPoint);
	}

	for (auto MinorPoint : MinorPoints)
	{
		std::vector<SVertex>::iterator Iter = std::find(PairedMinorPoints.begin(), PairedMinorPoints.end(), MinorPoint);
		if (Iter != PairedMinorPoints.end())
			continue;

		SVertex NearestPoint = __findNearestPoint(MajorPoints, MinorPoint);
		PairingPointsAmended.insert(std::pair<SVertex, SVertex>(NearestPoint, MinorPoint));
	}

	for (auto Iter = PairingPoints.begin(); Iter != PairingPoints.end(); ++Iter)
	{
		SVertex SharingVertex;
		if (PairingPointsAmended.find(Iter->first) != PairingPointsAmended.end())
			SharingVertex = __interpolatePoint(__interpolatePoint(Iter->first, Iter->second), __interpolatePoint(Iter->first, PairingPointsAmended.find(Iter->first)->second));
		else
			SharingVertex = __interpolatePoint(Iter->first, Iter->second);
		voPublicVertices.push_back(SharingVertex);
	}
}

//*****************************************************************
//FUNCTION: 
double CBasicMeshSuture::__computeDistance(const SVertex& vLHSVertex, const SVertex& vRHSVertex)
{
	return std::sqrt(std::pow(vLHSVertex.x - vRHSVertex.x, 2) + std::pow(vLHSVertex.y - vRHSVertex.y, 2) + std::pow(vLHSVertex.z - vRHSVertex.z, 2));
}

//*****************************************************************
//FUNCTION: 
hiveObliquePhotography::SVertex CBasicMeshSuture::__interpolatePoint(const SVertex& vLHSVertex, const SVertex& vRHSVertex)
{
	hiveObliquePhotography::SVertex NewVertex;
	NewVertex.x = 0.5 * (vLHSVertex.x + vRHSVertex.x);
	NewVertex.y = 0.5 * (vLHSVertex.y + vRHSVertex.y);
	NewVertex.z = 0.5 * (vLHSVertex.z + vRHSVertex.z);
	NewVertex.nx = 0.5 * (vLHSVertex.nx + vRHSVertex.nx);
	NewVertex.ny = 0.5 * (vLHSVertex.ny + vRHSVertex.ny);
	NewVertex.nz = 0.5 * (vLHSVertex.nz + vRHSVertex.nz);
	NewVertex.u = 0.5 * (vLHSVertex.u + vRHSVertex.u);
	NewVertex.v = 0.5 * (vLHSVertex.v + vRHSVertex.v);
	return NewVertex;
}

//*****************************************************************
//FUNCTION: 
hiveObliquePhotography::SVertex CBasicMeshSuture::__findNearestPoint(const std::vector<SVertex>& vVectexSet, const SVertex& vVertex)
{
	double MinDistance = FLT_MAX;
	SVertex NearestPoint;
	for (auto Point : vVectexSet)
	{
		double Distance = __computeDistance(vVertex, Point);
		if (Distance < MinDistance)
		{
			MinDistance = Distance;
			NearestPoint = Point;
		}
	}
	return NearestPoint;
}

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::__connectVerticesWithMesh(CMesh& vioMesh, std::vector<int>& vDissociatedIndices, std::vector<SVertex>& vPublicVertices)
{
	_ASSERTE(!vDissociatedIndices.empty() && !vPublicVertices.empty());

	std::vector<int> PublicIndices;
	PublicIndices.reserve(vPublicVertices.size());
	for (size_t i = 0; i < vPublicVertices.size(); ++i)
	{
		vioMesh.m_Vertices.push_back(vPublicVertices[i]);
		PublicIndices.push_back(i + vioMesh.m_Vertices.size());
	}

	auto ConnectionFaceSet = __genConnectionFace(vDissociatedIndices.size(), PublicIndices.size(), true, true);	// order is heuristic

	for (auto Offset = vDissociatedIndices.size(); auto& Face : ConnectionFaceSet)
	{
		SFace FaceWithMeshIndex;
		for (int i = 0; i < 3; i++)
			FaceWithMeshIndex[i] = Face[i] < Offset ? vDissociatedIndices[Face[i]] : PublicIndices[Face[i] - Offset];
		vioMesh.m_Faces.push_back(FaceWithMeshIndex);
	}
}

//*****************************************************************
//FUNCTION: 
std::vector<hiveObliquePhotography::SFace> CBasicMeshSuture::__genConnectionFace(IndexType vNumLeft, IndexType vNumRight, bool vLeftBeforeRight, bool vIsClockwise)
{
	if (!vIsClockwise)
		return __genConnectionFace(vNumRight, vNumLeft, !vLeftBeforeRight, !vIsClockwise);

	std::vector<SFace> ConnectionFaceSet;
	std::pair<IndexType, IndexType> Offset(0, 0);
	if (vLeftBeforeRight)
		Offset.second = vNumLeft;
	else
		Offset.first = vNumRight;
	
	for (IndexType LeftCursor = 0, RightCursor = 0; LeftCursor < vNumLeft && RightCursor < vNumRight; )
	{
		auto LeftWithOffset = LeftCursor + Offset.first;
		auto RightWithOffset = RightCursor + Offset.second;

		if ((2 * LeftCursor + 1) * (vNumRight - 1) < (2 * RightCursor + 1) * (vNumLeft - 1))
		{
			if (LeftCursor + 1 >= vNumLeft)
				break;

			ConnectionFaceSet.emplace_back(LeftWithOffset, RightWithOffset, LeftWithOffset + 1);
			++LeftCursor;
		}
		else
		{
			if (RightCursor + 1 >= vNumRight)
				break;

			ConnectionFaceSet.emplace_back(LeftWithOffset, RightWithOffset, RightWithOffset + 1);
			++RightCursor;
		}
	}
	return ConnectionFaceSet;
}

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::__removeUnreferencedVertex(CMesh& vioMesh)
{
	CVcgMesh VcgMesh;
	toVcgMesh(vioMesh, VcgMesh);
	vcg::tri::Clean<CVcgMesh>::RemoveUnreferencedVertex(VcgMesh);
	fromVcgMesh(VcgMesh, vioMesh);
}
