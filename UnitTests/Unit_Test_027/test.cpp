#include "pch.h"

#include "SceneReconstructionConfig.h"
#include "IntersectMeshAndPlane.h"
#include "ObliquePhotographyDataInterface.h"

using namespace hiveObliquePhotography::SceneReconstruction;

//测试用例列表：
//  * Test_NoIntersection: 测试平面与网格不相交
//  * Test_SeveralIntersections: 测试平面与网格有多个交点（在顶点处相交，在边处相交）
//  * Test_TerrainModelIntersections: 测试平面与特定复杂地形网格相交
//  * Test_SortByVertexLoop: 测试游离点loop排序正确性

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

	hiveObliquePhotography::CMesh Mesh;
	Eigen::Vector4f Plane(1.0, 0.0, 0.0, 3.0);
	std::vector<hiveObliquePhotography::SVertex> IntersectionPoints;
	std::vector<int> DissociatedIndices;
	intersectMeshAndPlane(Plane, Mesh, IntersectionPoints, DissociatedIndices);

	EXPECT_EQ(IntersectionPoints.size(), 0);
}

TEST_F(TestMeshPlaneIntersection, Test_SeveralIntersections)
{
	LoadMesh(SimpleMeshPath);
	hiveObliquePhotography::CMesh Mesh;
	std::vector<hiveObliquePhotography::SVertex> IntersectionPoints;
	

	Eigen::Vector4f PlaneWithOneIntersection(1.0, 0.0, 0.0, -2.0);
	Mesh = m_Mesh;
	std::vector<int> DissociatedIndices;
	intersectMeshAndPlane(PlaneWithOneIntersection, Mesh, IntersectionPoints, DissociatedIndices);
	EXPECT_EQ(IntersectionPoints.size(), 1);

	Eigen::Vector4f PlaneWithSeveralIntersections(1.0, 0.0, 0.0, -1.5);
	Mesh = m_Mesh;
	intersectMeshAndPlane(PlaneWithSeveralIntersections, Mesh, IntersectionPoints, DissociatedIndices);
	EXPECT_EQ(IntersectionPoints.size(), 2);

	Eigen::Vector4f PlaneWithSeveralIntersectionsOverlapWithOrigin(-1.0, 0.0, 0.0, 0.0);
	Mesh = m_Mesh;
	intersectMeshAndPlane(PlaneWithSeveralIntersectionsOverlapWithOrigin, Mesh, IntersectionPoints, DissociatedIndices);
	EXPECT_EQ(IntersectionPoints.size(), 3);

}

TEST_F(TestMeshPlaneIntersection, Test_TerrainModel)
{
	LoadMesh(SceneMeshPath);
	hiveObliquePhotography::CMesh Mesh;
	std::vector<hiveObliquePhotography::SVertex> IntersectionPoints;
	Eigen::Vector4f Plane(1.0, 0.0, 0.0, 132.0);
	Mesh = m_Mesh;
	std::vector<int> DissociatedIndices;
	intersectMeshAndPlane(Plane, Mesh, IntersectionPoints, DissociatedIndices);
	EXPECT_EQ(IntersectionPoints.size(), 204);
}
