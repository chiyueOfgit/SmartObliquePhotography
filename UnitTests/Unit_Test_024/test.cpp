#include "pch.h"
#include "RayCastingBaker.h"
#include "SceneReconstructionConfig.h"

#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include "ObliquePhotographyDataInterface.h"

//测试用例列表：
//  * findTexelsPerFace: 测试每三角面片所覆盖的纹素是否正确
// 
//  * executeIntersection: 
// 
//  * calcTexelColor: 
// 

using namespace hiveObliquePhotography::SceneReconstruction;

const auto PlaneMeshPath = TESTMODEL_DIR + std::string("/Test024_Model/Plane/Plane100.obj");
const auto CloudPath = TESTMODEL_DIR + std::string("");

class TestCastingTextureBaker : public testing::Test
{
protected:
	void SetUp() override
	{
		_loadMesh(PlaneMeshPath, m_Mesh);

		m_pCloud = hiveObliquePhotography::hiveInitPointCloudScene({ CloudPath });

		m_pTextureBaker = _createBaker(m_Mesh);
	}

	void TearDown() override
	{
		delete m_pTextureBaker;
	}

	void _loadMesh(const std::string& vPath, hiveObliquePhotography::CMesh& vMesh)
	{
		pcl::TextureMesh TexMesh1, TexMesh2;
		pcl::io::loadOBJFile(vPath, TexMesh1);
		pcl::io::loadPolygonFileOBJ(vPath, TexMesh2);
		TexMesh2.tex_materials = TexMesh1.tex_materials;
		vMesh = hiveObliquePhotography::CMesh(TexMesh2);
		ASSERT_TRUE(!vMesh.m_Vertices.empty());
		ASSERT_TRUE(!vMesh.m_Faces.empty());
	}

	CRayCastingBaker* _createBaker(const hiveObliquePhotography::CMesh& vMesh)
	{
		return hiveDesignPattern::hiveCreateProduct<CRayCastingBaker>(KEYWORD::RAYCASTING_TEXTUREBAKER, CSceneReconstructionConfig::getInstance()->getSubConfigByName("RayCasting"), vMesh);
	}

	hiveObliquePhotography::CMesh m_Mesh;
	PointCloud_t::Ptr m_pCloud = nullptr;

	CRayCastingBaker* m_pTextureBaker = nullptr;
};

TEST_F(TestCastingTextureBaker, TestFindTexelsPerFace)
{

}

TEST_F(TestCastingTextureBaker, TestExecuteIntersection)
{

}

TEST_F(TestCastingTextureBaker, TestCalcTexelColor)
{

}

