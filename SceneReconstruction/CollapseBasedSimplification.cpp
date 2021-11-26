#include "pch.h"
#include "CollapseBasedSimplification.h"
#include "VcgMesh.hpp"
#include <igl/decimate.h>
#include <vcg/complex/algorithms/clean.h>

using namespace hiveObliquePhotography::SceneReconstruction;

_REGISTER_NORMAL_PRODUCT(CCollapseBasedSimplification, KEYWORD::COLLAPSE_BASED_SIMPLIFICATION)

//*****************************************************************
//FUNCTION: 
void CCollapseBasedSimplification::__toManifold(CMesh& vioMesh)         
{
	_ASSERTE(!(vioMesh.m_Vertices.empty() || vioMesh.m_Faces.empty()));
	CVcgMesh VcgMesh;
	toVcgMesh(vioMesh, VcgMesh);                                   
	vcg::tri::Clean<CVcgMesh>::SplitNonManifoldVertex(VcgMesh, 0.1);
	vcg::tri::Allocator<CVcgMesh>::CompactFaceVector(VcgMesh);
	vcg::tri::Allocator<CVcgMesh>::CompactVertexVector(VcgMesh);
	hiveObliquePhotography::fromVcgMesh(VcgMesh, vioMesh);
}

//*****************************************************************
//FUNCTION: 
hiveObliquePhotography::CMesh CCollapseBasedSimplification::simplifyMesh()     
{
	__toManifold(m_Mesh);

	Eigen::MatrixXd V = m_Mesh.getVerticesMatrix();
	Eigen::MatrixXi F = m_Mesh.getFacesMatrix();
	Eigen::VectorXi J, I;
	Eigen::MatrixXd OV;
	Eigen::MatrixXi OF;
	bool a = igl::decimate(V, F, 0.9 * F.rows(), OV, OF, J, I);
	hiveObliquePhotography::CMesh SimplifiedMesh(OV, OF);
	auto VertexNormals = __calcVertexNormal(SimplifiedMesh);
	for (int i = 0; i < VertexNormals.size(); i++)
	{
		SimplifiedMesh.m_Vertices[i].nx = VertexNormals[i].x();
		SimplifiedMesh.m_Vertices[i].ny = VertexNormals[i].y();
		SimplifiedMesh.m_Vertices[i].nz = VertexNormals[i].z();
	}
	return SimplifiedMesh;
}

std::vector<Eigen::Vector3f> CCollapseBasedSimplification::__calcVertexNormal(const CMesh& vMesh)
{
	_ASSERTE(!vMesh.m_Vertices.empty());
	Eigen::Vector3f HeightAxis = { 0.0f ,0.0f, 1.0f };
	std::vector<Eigen::Vector3f> VertexNormals(vMesh.m_Vertices.size(), { 0.0f, 0.0f, 0.0f });
	std::vector<int> NumNormals(vMesh.m_Vertices.size(), 0);
	for (auto& Face : vMesh.m_Faces)
	{
		auto VertexA = vMesh.m_Vertices[Face.a];
		auto VertexB = vMesh.m_Vertices[Face.b];
		auto VertexC = vMesh.m_Vertices[Face.c];

		auto VectorAB = VertexB.xyz() - VertexA.xyz();
		auto VectorBC = VertexC.xyz() - VertexB.xyz();

		Eigen::Vector3f FaceNormal = (VectorAB.cross(VectorBC)).normalized();
		if (FaceNormal.dot(HeightAxis) < 0)
			FaceNormal = -FaceNormal;

		for (int i = 0; i < 3; i++)
		{
			VertexNormals[Face[i]] += FaceNormal;
			NumNormals[Face[i]]++;
		}
	}
	for (int i = 0; i < VertexNormals.size(); i++)
		VertexNormals[i] /= NumNormals[i]; 
	return VertexNormals;
}