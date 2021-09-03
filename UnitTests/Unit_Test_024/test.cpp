#include "pch.h"
#include "RayCastingBaker.h"
#include "SceneReconstructionConfig.h"

#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include "ObliquePhotographyDataInterface.h"

//测试用例列表：
//  * findTexelsPerFace: 测试每三角面片所覆盖的纹素是否正确
// 
//  * executeIntersection: 测试执行光线投射后与点云面片相交结果是否正确
//  * TestExecuteIntersection_1: 测试点云点在面片法线一侧的相交情况
//  * TestExecuteIntersection_2: 测试点云点在面片法线两侧的相交情况
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

TEST_F(TestCastingTextureBaker, TestExecuteIntersection_1)
{
	auto pCloud = hiveObliquePhotography::hiveInitPointCloudScene({ TESTMODEL_DIR + std::string("/Test024_Model/TestPointCloud.ply") });
	m_pTextureBaker->setPointCloud(pCloud);
	
	STexelInfo TestTexel{ {98,98},{48.0f, 0.0f, 48.0f},2 };
	auto CandidateSet = m_pTextureBaker->executeIntersection(TestTexel);
	EXPECT_EQ(CandidateSet.size(), 3);
	sort(CandidateSet.begin(), CandidateSet.end(), [](SCandidateInfo& vA, SCandidateInfo& vB) {return vA.PointIndex < vB.PointIndex; });
	Eigen::Vector3f IntersectionOne{ 48.0f, 2.0f, 48.0f };
	Eigen::Vector3f IntersectionTwo{ 48.0f, 3.0f, 48.0f };
	EXPECT_EQ(CandidateSet[0].PointIndex, 0);
	EXPECT_EQ(CandidateSet[0].Pos, IntersectionOne);
	EXPECT_EQ(CandidateSet[1].PointIndex, 1);
	EXPECT_EQ(CandidateSet[1].Pos, IntersectionTwo);
	EXPECT_EQ(CandidateSet[2].PointIndex, 2);
	EXPECT_EQ(CandidateSet[2].Pos, IntersectionOne);
}

TEST_F(TestCastingTextureBaker, TestExecuteIntersection_2)
{
	auto pCloud = hiveObliquePhotography::hiveInitPointCloudScene({ TESTMODEL_DIR + std::string("/Test024_Model/TestPointCloud.ply") });
	m_pTextureBaker->setPointCloud(pCloud);

	STexelInfo TestTexel{ {51,51},{1.5f, 0.0f, 1.5f},3 };
	auto CandidateSet = m_pTextureBaker->executeIntersection(TestTexel);
	EXPECT_EQ(CandidateSet.size(), 2);
	sort(CandidateSet.begin(), CandidateSet.end(), [](SCandidateInfo& vA, SCandidateInfo& vB) {return vA.PointIndex < vB.PointIndex; });
	Eigen::Vector3f IntersectionOne{ 1.5f, 2.0f, 1.5f };
	Eigen::Vector3f IntersectionTwo{ 1.5f, -2.0f, 1.5f };
	EXPECT_EQ(CandidateSet[0].PointIndex, 3);
	EXPECT_EQ(CandidateSet[0].Pos, IntersectionOne);
	EXPECT_EQ(CandidateSet[1].PointIndex, 4);
	EXPECT_EQ(CandidateSet[1].Pos, IntersectionTwo);
}

TEST_F(TestCastingTextureBaker, TestCalcTexelColor)
{

}

