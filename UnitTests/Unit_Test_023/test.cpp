#include "pch.h"
#include "RayCastingBaker.h"
#include "SceneReconstructionConfig.h"

#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include "ObliquePhotographyDataInterface.h"

//≤‚ ‘”√¿˝¡–±Ì£∫
//  * findTexelsPerFace: 
// 
//  * executeIntersection: 
// 
//  * calcTexelColor: 
// 

using namespace hiveObliquePhotography::SceneReconstruction;

const auto MeshPath = TESTMODEL_DIR + std::string("");
const auto CloudPath = TESTMODEL_DIR + std::string("");

class TestCastingTextureBaker : public testing::Test
{
protected:
	void SetUp() override
	{
		pcl::TextureMesh TexMesh1, TexMesh2;
		pcl::io::loadOBJFile(MeshPath, TexMesh1);
		pcl::io::loadPolygonFileOBJ(MeshPath, TexMesh2);
		TexMesh2.tex_materials = TexMesh1.tex_materials;
		m_Mesh = hiveObliquePhotography::CMesh(TexMesh2);
		ASSERT_TRUE(!m_Mesh.m_Vertices.empty());
		ASSERT_TRUE(!m_Mesh.m_Faces.empty());

		m_pCloud = hiveObliquePhotography::hiveInitPointCloudScene({ CloudPath });
		ASSERT_NE(m_pCloud, nullptr);

		m_pTextureBaker = hiveDesignPattern::hiveCreateProduct<CRayCastingBaker>(KEYWORD::RAYCASTING_TEXTUREBAKER, CSceneReconstructionConfig::getInstance()->getSubConfigByName("RayCasting"), m_Mesh);
		ASSERT_NE(m_pTextureBaker, nullptr);
	}

	void TearDown() override
	{
		delete m_pTextureBaker;
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

