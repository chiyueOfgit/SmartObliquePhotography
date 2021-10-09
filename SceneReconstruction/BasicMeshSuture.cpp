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
	std::vector<SVertex> LHSIntersectionPoints, RHSIntersectionPoints;
	__executeIntersection(m_MeshLHS, SegmentPlane, LHSDissociatedIndices, LHSIntersectionPoints);
	__executeIntersection(m_MeshRHS, SegmentPlane, RHSDissociatedIndices, RHSIntersectionPoints);

	auto PublicVertices = __generatePublicVertices(LHSIntersectionPoints, RHSIntersectionPoints);
	__connectVerticesWithMesh(m_MeshLHS, LHSDissociatedIndices, PublicVertices);
	__connectVerticesWithMesh(m_MeshRHS, RHSDissociatedIndices, PublicVertices);

	__removeUnreferencedVertex(m_MeshLHS);
	__removeUnreferencedVertex(m_MeshRHS);
}

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
std::vector<hiveObliquePhotography::SVertex> CBasicMeshSuture::__generatePublicVertices(std::vector<SVertex>& vLHSIntersectionPoints, std::vector<SVertex>& vRHSIntersectionPoints)
{
	return {};
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

	auto ConnectionFaceSet = __genConnectionFace(vDissociatedIndices.size(), PublicIndices.size(), true);	// order is heuristic

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
std::vector<hiveObliquePhotography::SFace> CBasicMeshSuture::__genConnectionFace(int vNumLeft, int vNumRight, bool vDefaultOrder)
{
	std::vector<SFace> ConnectionFaceSet;

	int NumLess = vNumLeft < vNumRight ? vNumLeft : vNumRight;
	int NumMore = vNumLeft < vNumRight ? vNumRight : vNumLeft;
	float ContainRate = float(NumMore) / NumLess;

	int LessOffset = NumLess == vNumLeft ? 0 : vNumLeft;
	int MoreOffset = NumMore == vNumRight ? vNumLeft : 0;
	IndexType LessCursor = LessOffset, MoreCursor = MoreOffset;
	IndexType LessEnd = NumLess + LessOffset, MoreEnd = NumMore + MoreOffset;

	auto genFixLessFace = [&](IndexType vLess, IndexType& vMore)
	{
		if (vDefaultOrder)
			ConnectionFaceSet.emplace_back(vLess, vMore, vMore + 1);
		else
			ConnectionFaceSet.emplace_back(vLess, vMore + 1, vMore);
		++vMore;
	};
	auto genFixMoreFace = [&](IndexType& vLess, IndexType vMore)
	{
		if (vDefaultOrder)
			ConnectionFaceSet.emplace_back(vLess, vMore, vLess + 1);
		else
			ConnectionFaceSet.emplace_back(vLess, vLess + 1, vMore);
		++vLess;
	};

	int CheckPoint = 1;
	float AccumContain = ContainRate;
	while (LessCursor + 1 < LessEnd && MoreCursor + 1 < MoreEnd)
	{
		while (CheckPoint < AccumContain && MoreCursor + 1 < MoreEnd)
		{
			genFixLessFace(LessCursor, MoreCursor);
			CheckPoint++;
		}
		AccumContain += ContainRate;

		genFixMoreFace(LessCursor, MoreCursor);
	}

	while (LessCursor + 1 < LessEnd)
		genFixMoreFace(LessCursor, MoreCursor);
	while (MoreCursor + 1 < MoreEnd)
		genFixLessFace(LessCursor, MoreCursor);

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
