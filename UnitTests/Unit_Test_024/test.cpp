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
		m_Mesh = _loadMesh(PlaneMeshPath);

		m_pCloud = hiveObliquePhotography::hiveInitPointCloudScene({ CloudPath });

		m_pTextureBaker = _createBaker(m_Mesh);
	}

	void TearDown() override
	{
		delete m_pTextureBaker;
	}

	hiveObliquePhotography::CMesh _loadMesh(const std::string& vPath)
	{
		pcl::TextureMesh TexMesh1, TexMesh2;
		pcl::io::loadOBJFile(vPath, TexMesh1);
		pcl::io::loadPolygonFileOBJ(vPath, TexMesh2);
		TexMesh2.tex_materials = TexMesh1.tex_materials;
		hiveObliquePhotography::CMesh Mesh(TexMesh2);
		bool EmptyFlag = Mesh.m_Vertices.empty() || Mesh.m_Faces.empty();
		EXPECT_FALSE(EmptyFlag);
		if (EmptyFlag)
			std::cerr << "mesh load error." << std::endl;
		return Mesh;
	}

	CRayCastingBaker* _createBaker(const hiveObliquePhotography::CMesh& vMesh)
	{
		auto pBaker =  hiveDesignPattern::hiveCreateProduct<CRayCastingBaker>(KEYWORD::RAYCASTING_TEXTUREBAKER, CSceneReconstructionConfig::getInstance()->getSubConfigByName("RayCasting"), vMesh);
		EXPECT_NE(pBaker, nullptr);
		if (!pBaker)
			std::cerr << "create baker error." << std::endl;
		return pBaker;
	}

	hiveObliquePhotography::CMesh m_Mesh;
	PointCloud_t::Ptr m_pCloud = nullptr;

	CRayCastingBaker* m_pTextureBaker = nullptr;
};

TEST_F(TestCastingTextureBaker, TestFindTexelsPerFace)
{
	//8 faces plane
	m_Mesh = _loadMesh(PlaneMeshPath);
	_createBaker(m_Mesh);
	for (auto& Face : m_Mesh.m_Faces)
	{
		Eigen::Vector2i Resolution{ 512, 512 };
		int NumTexels = Resolution.x() * Resolution.y();
		auto TexelInfos = m_pTextureBaker->findTexelsPerFace(Face, Resolution);
		ASSERT_TRUE(!TexelInfos.empty());
		EXPECT_NEAR(TexelInfos.size(), NumTexels * (1 / 8), NumTexels * 0.1);
		
		Eigen::Vector2i Resolution{ 10, 10 };

	}

}

TEST_F(TestCastingTextureBaker, TestExecuteIntersection)
{

}

TEST_F(TestCastingTextureBaker, TestCalcTexelColor)
{

}

