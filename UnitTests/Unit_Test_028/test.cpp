#include "pch.h"
#include <vcg/complex/algorithms/clean.h>
#include "SceneReconstructionConfig.h"
#include "ObliquePhotographyDataInterface.h"
#include "VcgMesh.hpp"

//测试用例列表：
//  * Test_NoHoles: 判断网格是否有洞
//	* Test_NoFlips: 判断网格是否有面片翻转
//	* Test_NoDuplicates: 判断网格是否有面片重复

using namespace hiveObliquePhotography::SceneReconstruction;

const auto LhsMeshPath = TESTMODEL_DIR + std::string("Test028_Model/005006.obj");
const auto RhsMeshPath = TESTMODEL_DIR + std::string("Test028_Model/006006.obj");

class TestMeshLegality : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void LoadMesh(const std::string& vPath)
	{
		hiveObliquePhotography::hiveLoadMeshModel(m_Mesh, vPath);
		toVcgMesh(m_Mesh, m_VcgMesh);
	}

	void LoadTwoMeshes(const std::string& vLhsPath, const std::string& vRhsPath)
	{
		//hiveObliquePhotography::hiveLoadMeshModel(m_LhsMesh, vLhsPath);
		//hiveObliquePhotography::hiveLoadMeshModel(m_RhsMesh, vRhsPath);

		////合并为一个VcgMesh
		//m_VcgMesh.Clear();

		//auto VertexIterator = vcg::tri::Allocator<hiveObliquePhotography::CVcgMesh>::AddVertices(m_VcgMesh, m_LhsMesh.m_Vertices.size());
		//for (auto& Vertex : m_LhsMesh.m_Vertices)
		//{
		//	VertexIterator->P() = { Vertex.x, Vertex.y, Vertex.z };
		//	VertexIterator->N() = { Vertex.nx, Vertex.ny, Vertex.nz };
		//	VertexIterator->T() = { Vertex.u, Vertex.v };
		//	++VertexIterator;
		//}

		//auto FaceIterator = vcg::tri::Allocator<hiveObliquePhotography::CVcgMesh>::AddFaces(m_VcgMesh, m_LhsMesh.m_Faces.size());
		//for (auto& Face : m_LhsMesh.m_Faces)
		//{
		//	FaceIterator->V(0) = &m_VcgMesh.vert.at(Face.a);
		//	FaceIterator->V(1) = &m_VcgMesh.vert.at(Face.b);
		//	FaceIterator->V(2) = &m_VcgMesh.vert.at(Face.c);
		//	++FaceIterator;
		//}

		//vcg::tri::UpdateTopology<hiveObliquePhotography::CVcgMesh>::FaceFace(m_VcgMesh);
	}

	hiveObliquePhotography::CMesh m_Mesh;
	hiveObliquePhotography::CMesh m_LhsMesh;
	hiveObliquePhotography::CMesh m_RhsMesh;
	hiveObliquePhotography::CVcgMesh m_VcgMesh;
};

TEST_F(TestMeshLegality, Test_NoHoles)
{
	LoadMesh(LhsMeshPath);
	int HolesCount = vcg::tri::Clean<hiveObliquePhotography::CVcgMesh>::CountHoles(m_VcgMesh);
	EXPECT_EQ(HolesCount, 0);
}

TEST_F(TestMeshLegality, Test_NoFlips)
{
	LoadMesh(LhsMeshPath);
	int FlipsCount = vcg::tri::Clean<hiveObliquePhotography::CVcgMesh>::RemoveFaceFoldByFlip(m_VcgMesh);
	EXPECT_EQ(FlipsCount, 0);
}

TEST_F(TestMeshLegality, Test_NoDuplicates)
{
	LoadMesh(LhsMeshPath);
	int OverlapsCount = vcg::tri::Clean<hiveObliquePhotography::CVcgMesh>::RemoveDuplicateFace(m_VcgMesh);
	EXPECT_EQ(OverlapsCount, 0);
}