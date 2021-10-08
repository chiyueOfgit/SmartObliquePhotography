#include "pch.h"

#include "SceneReconstructionConfig.h"
#include "MeshPlaneIntersection.h"
#include "ObliquePhotographyDataInterface.h"

using namespace hiveObliquePhotography::SceneReconstruction;

const auto SimpleMeshPath = TESTMODEL_DIR + std::string("Test027_Model/SimpleMesh.obj");
const auto SceneMeshPath = TESTMODEL_DIR + std::string("Test027_Model/005006.obj");

class TestMeshPlaneIntersection : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void LoadMesh(const std::string& vPath)
	{
		hiveObliquePhotography::hiveLoadMeshModel(m_Mesh, vPath);
	}

	hiveObliquePhotography::CMesh m_Mesh;
};

TEST_F(TestMeshPlaneIntersection, Test_NoIntersection)
{
	LoadMesh(SimpleMeshPath);

	Eigen::Vector4f Plane(1.0, 0.0, 0.0, 3.0);
	hiveObliquePhotography::CMesh DissociatedMesh;
	std::vector<hiveObliquePhotography::SVertex> IntersectionPoints;
	CMeshPlaneIntersection MeshPlaneIntersection;
	MeshPlaneIntersection.execute(m_Mesh, Plane);
	MeshPlaneIntersection.dumpDissociatedMesh(DissociatedMesh);
	MeshPlaneIntersection.dumpIntersectionPoints(IntersectionPoints);
	EXPECT_EQ(DissociatedMesh.m_Vertices.size(), 0);
	EXPECT_EQ(IntersectionPoints.size(), 0);
}

TEST_F(TestMeshPlaneIntersection, Test_SeveralIntersections)
{
	LoadMesh(SimpleMeshPath);
	hiveObliquePhotography::CMesh DissociatedMesh;
	std::vector<hiveObliquePhotography::SVertex> IntersectionPoints;
	CMeshPlaneIntersection MeshPlaneIntersection;

	Eigen::Vector4f PlaneWithOneIntersection(1.0, 0.0, 0.0, 2.0);
	MeshPlaneIntersection.execute(m_Mesh, PlaneWithOneIntersection);
	MeshPlaneIntersection.dumpDissociatedMesh(DissociatedMesh);
	MeshPlaneIntersection.dumpIntersectionPoints(IntersectionPoints);
	EXPECT_EQ(DissociatedMesh.m_Vertices.size(), 0);
	EXPECT_EQ(IntersectionPoints.size(), 1);

	Eigen::Vector4f PlaneWithSeveralIntersections(1.0, 0.0, 0.0, 1.5);
	MeshPlaneIntersection.execute(m_Mesh, PlaneWithSeveralIntersections);
	MeshPlaneIntersection.dumpDissociatedMesh(DissociatedMesh);
	MeshPlaneIntersection.dumpIntersectionPoints(IntersectionPoints);
	EXPECT_EQ(DissociatedMesh.m_Vertices.size(), 1);
	EXPECT_EQ(IntersectionPoints.size(), 2);

	Eigen::Vector4f PlaneWithSeveralIntersectionsOverlapWithOrigin(1.0, 0.0, 0.0, 0.0);
	MeshPlaneIntersection.execute(m_Mesh, PlaneWithSeveralIntersectionsOverlapWithOrigin);
	MeshPlaneIntersection.dumpDissociatedMesh(DissociatedMesh);
	MeshPlaneIntersection.dumpIntersectionPoints(IntersectionPoints);
	EXPECT_EQ(DissociatedMesh.m_Vertices.size(), 3);
	EXPECT_EQ(IntersectionPoints.size(), 3);

}

TEST_F(TestMeshPlaneIntersection, Test_TerrainModel)
{
	LoadMesh(SceneMeshPath);
	hiveObliquePhotography::CMesh DissociatedMesh;
	std::vector<hiveObliquePhotography::SVertex> IntersectionPoints;
	CMeshPlaneIntersection MeshPlaneIntersection;

	Eigen::Vector4f Plane(1.0, 0.0, 0.0, -132.0);
	MeshPlaneIntersection.execute(m_Mesh, Plane);
	MeshPlaneIntersection.dumpDissociatedMesh(DissociatedMesh);
	MeshPlaneIntersection.dumpIntersectionPoints(IntersectionPoints);
	EXPECT_EQ(DissociatedMesh.m_Vertices.size(), 153);
	EXPECT_EQ(IntersectionPoints.size(), 433);
}