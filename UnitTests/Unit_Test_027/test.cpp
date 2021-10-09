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

//TEST_F(TestMeshPlaneIntersection, Test_NoIntersection)
//{
//	LoadMesh(SimpleMeshPath);
//
//	hiveObliquePhotography::CMesh Mesh;
//	Eigen::Vector4f Plane(1.0, 0.0, 0.0, 3.0);
//	std::vector<hiveObliquePhotography::SVertex> IntersectionPoints;
//	CMeshPlaneIntersection MeshPlaneIntersection;
//	Mesh = m_Mesh;
//	MeshPlaneIntersection.execute(Mesh, Plane);
//
//	MeshPlaneIntersection.dumpIntersectionPoints(IntersectionPoints);
//	EXPECT_EQ(IntersectionPoints.size(), 0);
//}
//
//TEST_F(TestMeshPlaneIntersection, Test_SeveralIntersections)
//{
//	LoadMesh(SimpleMeshPath);
//	hiveObliquePhotography::CMesh Mesh;
//	std::vector<hiveObliquePhotography::SVertex> IntersectionPoints;
//	CMeshPlaneIntersection MeshPlaneIntersection;
//
//	Eigen::Vector4f PlaneWithOneIntersection(1.0, 0.0, 0.0, -2.0);
//	Mesh = m_Mesh;
//	MeshPlaneIntersection.execute(Mesh, PlaneWithOneIntersection);
//	MeshPlaneIntersection.dumpIntersectionPoints(IntersectionPoints);
//	EXPECT_EQ(IntersectionPoints.size(), 1);
//
//	Eigen::Vector4f PlaneWithSeveralIntersections(1.0, 0.0, 0.0, -1.5);
//	Mesh = m_Mesh;
//	MeshPlaneIntersection.execute(Mesh, PlaneWithSeveralIntersections);
//	MeshPlaneIntersection.dumpIntersectionPoints(IntersectionPoints);
//	EXPECT_EQ(IntersectionPoints.size(), 2);
//
//	Eigen::Vector4f PlaneWithSeveralIntersectionsOverlapWithOrigin(-1.0, 0.0, 0.0, 0.0);
//	Mesh = m_Mesh;
//	MeshPlaneIntersection.execute(Mesh, PlaneWithSeveralIntersectionsOverlapWithOrigin);
//	MeshPlaneIntersection.dumpIntersectionPoints(IntersectionPoints);
//	EXPECT_EQ(IntersectionPoints.size(), 3);
//
//}
//
//TEST_F(TestMeshPlaneIntersection, Test_TerrainModel)
//{
//	LoadMesh(SceneMeshPath);
//	hiveObliquePhotography::CMesh Mesh;
//	std::vector<hiveObliquePhotography::SVertex> IntersectionPoints;
//	CMeshPlaneIntersection MeshPlaneIntersection;
//
//	Eigen::Vector4f Plane(1.0, 0.0, 0.0, 132.0);
//	Mesh = m_Mesh;
//	MeshPlaneIntersection.execute(Mesh, Plane);
//	MeshPlaneIntersection.dumpIntersectionPoints(IntersectionPoints);
//	EXPECT_EQ(IntersectionPoints.size(), 433);
//}

double polyeval(Eigen::VectorXd coeffs, double x) {
	double result = 0.0;
	for (int i = 0; i < coeffs.size(); i++) { result += coeffs[i] * pow(x, i); }
	return result;
}

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
	assert(xvals.size() == yvals.size());
	assert(order >= 1 && order <= xvals.size() - 1);
	Eigen::MatrixXd A(xvals.size(), order + 1);
	for (int i = 0; i < xvals.size(); i++) {
		A(i, 0) = 1.0;
	}
	for (int j = 0; j < xvals.size(); j++) {
		for (int i = 0; i < order; i++) {
			A(j, i + 1) = A(j, i) * xvals(j);
		}
	}
	auto Q = A.householderQr();
	auto result = Q.solve(yvals);
	return result;
}

TEST(Test_TerrainModel)
{
	Eigen::VectorXd xvals(6);
	Eigen::VectorXd yvals(6);
	xvals << 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7;
	yvals << 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1;
	auto coeffs = polyfit(xvals, yvals, 10);
	std::cout << "Y(0.5)=" << polyeval(coeffs, 0.5) << endl;
	std::cout << "Y(1.0)=" << polyeval(coeffs, 1.0) << endl;
	std::cout << "Y(1.5)=" << polyeval(coeffs, 1.5) << endl;
	std::cout << "Y(2.0)=" << polyeval(coeffs, 2.0) << endl;
	std::cout << "Y(2.5)=" << polyeval(coeffs, 2.5) << endl;
	std::cout << "Y(3.0)=" << polyeval(coeffs, 3.0) << endl;
	std::cout << "Y(3.5)=" << polyeval(coeffs, 3.5) << endl;
	std::cout << "Y(4.0)=" << polyeval(coeffs, 4.0) << endl;
	std::cout << "Y(4.5)=" << polyeval(coeffs, 4.5) << endl;
	std::cout << "Y(5.0)=" << polyeval(coeffs, 5.0) << endl;
	std::cout << "Y(5.5)=" << polyeval(coeffs, 5.5) << endl;
	std::cout << "Y(6.0)=" << polyeval(coeffs, 6.0) << endl;
	std::cout << "Y(6.5)=" << polyeval(coeffs, 6.5) << endl;
	std::cout << "Y(7.0)=" << polyeval(coeffs, 7.0) << endl;
	std::cout << "Y(7.5)=" << polyeval(coeffs, 7.5) << endl;
}
