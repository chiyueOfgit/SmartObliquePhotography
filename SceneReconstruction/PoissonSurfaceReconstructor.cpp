#include "pch.h"
#include "PoissonSurfaceReconstructor.h"
#include <pcl/surface/poisson.h>

using namespace hiveObliquePhotography::SceneReconstruction;

_REGISTER_NORMAL_PRODUCT(CPoissonSurfaceReconstructor, KEYWORD::POISSON_RECONSTRUCTOR)

//*****************************************************************
//FUNCTION: 
void CPoissonSurfaceReconstructor::constructSurface(CMesh& voMesh)
{
	pcl::PolygonMesh PolMesh;
	pcl::Poisson<pcl::PointNormal> Poisson;
	Poisson.setDepth(m_pConfig->getAttribute<int>(KEYWORD::OCTREE_DEPTH).value());
	Poisson.setInputCloud(m_pSceneCloud);
	Poisson.performReconstruction(PolMesh);
	voMesh = CMesh(PolMesh);
	auto VertexNormals = __calcVertexNormal(voMesh);
	for (int i = 0; i < VertexNormals.size(); i++)
	{
		voMesh.m_Vertices[i].nx = VertexNormals[i].x();
		voMesh.m_Vertices[i].ny = VertexNormals[i].y();
		voMesh.m_Vertices[i].nz = VertexNormals[i].z();
	}
}

std::vector<Eigen::Vector3f> CPoissonSurfaceReconstructor::__calcVertexNormal(const CMesh& vMesh)
{
	_ASSERTE(!vMesh.m_Vertices.empty());
	std::pair<int, int> XY;
	int Height;
	vMesh.calcModelPlaneAxis(XY, Height);
	Eigen::Vector3f HeightAxis = { 0.0f ,0.0f, 0.0f };
	HeightAxis.data()[Height] = 1.0f;
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
