#include "pch.h"
#include "Mesh.h"

using namespace hiveObliquePhotography::SceneReconstruction;

CMesh::CMesh(const pcl::PolygonMesh& vPolMesh)
{
	int NumVertices = vPolMesh.cloud.width / vPolMesh.cloud.point_step;
	m_Vertices.resize(NumVertices);
	__fillVertices(m_Vertices, vPolMesh);

	
}

CMesh::CMesh(const pcl::TextureMesh& vTexMesh)
{
	int NumVertices = vTexMesh.cloud.width / vTexMesh.cloud.point_step;
	m_Vertices.resize(NumVertices);
	__fillVertices(m_Vertices, vTexMesh);
}

//*****************************************************************
//FUNCTION: 
pcl::PolygonMesh CMesh::toPolMesh(const CMesh& vMesh)
{

}

//*****************************************************************
//FUNCTION: 
pcl::TextureMesh CMesh::toTexMesh(const CMesh& vMesh)
{

}

//*****************************************************************
//FUNCTION: 