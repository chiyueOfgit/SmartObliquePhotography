#include "pch.h"
#include"BasicMeshSuture.h"
#include"MeshPlaneIntersection.h"

using namespace hiveObliquePhotography::SceneReconstruction;

_REGISTER_NORMAL_PRODUCT(CBasicMeshSuture, KEYWORD::BASIC_MESH_SUTURE)

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::sutureMeshes()
{
	Eigen::Vector4f SegmentPlane = __calcSegmentPlane();
	
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
Eigen::Vector4f CBasicMeshSuture::__calcSegmentPlane()
{
	
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
	
}

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::__connectVerticesWithMesh(CMesh& vioMesh, std::vector<int>& vDissociatedIndices, std::vector<SVertex>& vPublicVertices)
{
	
}

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::__removeUnreferencedVertex(CMesh& vioMesh)
{
	
}