#include "pch.h"
#include "ArapParameterizer.h"
#include "SceneReconstructionConfig.h"
#include "ObliquePhotographyDataInterface.h"
#include "VcgMesh.hpp"
#include <igl/decimate.h>
#include <vcg/complex/algorithms/clean.h>

//测试用例列表：
//   * findBoundaryPoint: 测试在简单场景下寻找边界点正确性。


using namespace hiveObliquePhotography::SceneReconstruction;

const auto PlaneMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Plane/Plane.obj");
const auto TileMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Tile_low/005-004.obj");
const auto StoneMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Others/LI_Rock_Pavers.obj");
const auto MountainMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Others/mountain.obj");
const auto ScuMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Scu/Tile16.obj");
const auto PyramidMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Pyramid.obj");
const auto PoissonMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Tile1.obj");

class TestArapParameterization : public testing::Test
{
protected:
	void SetUp() override
	{
		m_MeshPath = PlaneMeshPath;
		_loadObj(m_MeshPath, m_Mesh);
		ASSERT_TRUE(!m_Mesh.m_Vertices.empty());
		m_pMeshParameterization = _createProduct(m_Mesh);
	}

	void TearDown() override
	{
		delete m_pMeshParameterization;
	}

	void _loadObj(const std::string & vPath, hiveObliquePhotography::CMesh& voMesh)
	{
		hiveObliquePhotography::hiveLoadMeshModel(voMesh, vPath);
	}

	void _saveObj(const std::string& vPath, const hiveObliquePhotography::CMesh& vMesh)
	{
		hiveObliquePhotography::hiveSaveMeshModel(vMesh, vPath);
	}

	CArapParameterizer* _createProduct(const hiveObliquePhotography::CMesh& vMesh)
	{
		auto pParameterization =  hiveDesignPattern::hiveCreateProduct<IMeshParameterizer>(KEYWORD::ARAP_MESH_PARAMETERIZATION, CSceneReconstructionConfig::getInstance()->getSubConfigByName("Parameterization"), vMesh);
		EXPECT_NE(pParameterization, nullptr);
		if (!pParameterization)
			std::cerr << "create baker error." << std::endl;
		return dynamic_cast<CArapParameterizer*>(pParameterization);
	}

	hiveObliquePhotography::CMesh m_Mesh;
	pcl::TexMaterial m_Material;
	std::string m_MeshPath;

	CArapParameterizer* m_pMeshParameterization = nullptr;
};


//TEST_F(TestArapParameterization, TestfindBoundaryPoint)
//{
//	auto UV = m_pMeshParameterization->execute();
//	EXPECT_EQ(UV.rows(), m_Mesh.m_Vertices.size());
//	for (int Row = 0; Row < UV.rows(); Row++)
//	{
//		m_Mesh.m_Vertices[Row].u = UV.row(Row).x();
//		m_Mesh.m_Vertices[Row].v = UV.row(Row).y();
//	}
//
//	std::string ObjName = "Plane.obj";
//	_saveObj(ObjName, m_Mesh);
//}

TEST_F(TestArapParameterization, Simplification)
{
	hiveObliquePhotography::CVcgMesh VcgMesh;
	hiveObliquePhotography::toVcgMesh(m_Mesh, VcgMesh);
	vcg::tri::Clean<hiveObliquePhotography::CVcgMesh>::SplitNonManifoldVertex(VcgMesh,0.1);
	hiveObliquePhotography::fromVcgMesh(VcgMesh, m_Mesh);

	Eigen::MatrixXd V = m_Mesh.getVerticesMatrix();
	Eigen::MatrixXi F = m_Mesh.getFacesMatrix();
	Eigen::VectorXi J, I;
	Eigen::MatrixXd OV;
	Eigen::MatrixXi OF;
	bool a = igl::decimate(V, F, 0.9 * F.rows(), OV, OF, J, I);
	hiveObliquePhotography::CMesh Mesh(OV, OF);
	std::string ObjName = "Simplification.obj";
	_saveObj(ObjName, Mesh);
}