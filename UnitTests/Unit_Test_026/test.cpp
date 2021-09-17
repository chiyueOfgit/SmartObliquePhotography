#include "pch.h"
#include "ArapParameterization.h"
#include "SceneReconstructionConfig.h"

#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>

//测试用例列表：
//   * findBoundaryPoint: 测试在简单场景下寻找边界点正确性。


using namespace hiveObliquePhotography::SceneReconstruction;

const auto PlaneMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Plane/Plane100.obj");

class TestArapParameterization : public testing::Test
{
protected:
	void SetUp() override
	{
		m_Mesh = _loadMesh(PlaneMeshPath);
		m_pMeshParameterization = _createProduct(m_Mesh);
	}

	void TearDown() override
	{
		delete m_pMeshParameterization;
	}

	hiveObliquePhotography::CMesh _loadMesh(const std::string& vPath)
	{
		pcl::TextureMesh TexMesh1, TexMesh2;
		pcl::io::loadOBJFile(vPath, TexMesh1);
		pcl::io::loadPolygonFileOBJ(vPath, TexMesh2);
		TexMesh2.tex_materials = TexMesh1.tex_materials;
		hiveObliquePhotography::CMesh Mesh(TexMesh1);
		bool EmptyFlag = Mesh.m_Vertices.empty() || Mesh.m_Faces.empty();
		EXPECT_FALSE(EmptyFlag);
		if (EmptyFlag)
			std::cerr << "mesh load error." << std::endl;
		return Mesh;
	}

	CArapParameterization* _createProduct(const hiveObliquePhotography::CMesh& vMesh)
	{
		auto pParameterization =  hiveDesignPattern::hiveCreateProduct<CArapParameterization>(KEYWORD::ARAP_MESH_PARAMETERIZATION, CSceneReconstructionConfig::getInstance()->getSubConfigByName("RayCasting"), vMesh);
		EXPECT_NE(pParameterization, nullptr);
		if (!pParameterization)
			std::cerr << "create baker error." << std::endl;
		return pParameterization;
	}

	hiveObliquePhotography::CMesh m_Mesh;

	CArapParameterization* m_pMeshParameterization = nullptr;
};


TEST_F(TestArapParameterization, TestfindBoundaryPoint)
{
	auto PointSet = m_pMeshParameterization->findBoundaryPoint();
	EXPECT_EQ(PointSet.size(), 9);
	auto UV = m_pMeshParameterization->execute();

	EXPECT_EQ(UV.rows(), m_Mesh.m_Vertices.size());
	for (int Row = 0; Row < UV.rows(); Row++)
	{
		m_Mesh.m_Vertices[Row].u = UV.row(Row)(0);
		m_Mesh.m_Vertices[Row].v = UV.row(Row)(1);
	}
	int i = 0;
}