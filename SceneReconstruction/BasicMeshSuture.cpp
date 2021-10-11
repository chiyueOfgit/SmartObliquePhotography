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
	_ASSERTE(m_SegmentPlane.norm());

	std::vector<int> LHSDissociatedIndices, RHSDissociatedIndices;
	std::vector<SVertex> LHSIntersectionPoints, RHSIntersectionPoints;
	__executeIntersection(m_MeshLHS, m_SegmentPlane, LHSDissociatedIndices, LHSIntersectionPoints);
	__executeIntersection(m_MeshRHS, m_SegmentPlane, RHSDissociatedIndices, RHSIntersectionPoints);

	auto PublicVertices = __generatePublicVertices(LHSIntersectionPoints, RHSIntersectionPoints);
	__connectVerticesWithMesh(m_MeshLHS, LHSDissociatedIndices, PublicVertices);
	__connectVerticesWithMesh(m_MeshRHS, RHSDissociatedIndices, PublicVertices);

	__removeUnreferencedVertex(m_MeshLHS);
	__removeUnreferencedVertex(m_MeshRHS);
}

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::setCloud4SegmentPlane(PointCloud_t::Ptr vLHSCloud, PointCloud_t::Ptr vRHSCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr TempOne(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr TempTwo(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*vLHSCloud, *TempOne);
	pcl::copyPointCloud(*vRHSCloud, *TempTwo);
	m_SegmentPlane = CFindSplitPlane().execute(TempOne, TempTwo);
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
