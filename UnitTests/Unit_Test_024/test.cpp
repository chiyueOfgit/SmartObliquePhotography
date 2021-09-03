#include "pch.h"
#include "RayCastingBaker.h"
#include "SceneReconstructionConfig.h"

#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include "ObliquePhotographyDataInterface.h"

//测试用例列表：
//  * findTexelsPerFace: 测试每三角面片所覆盖的纹素及对应世界坐标是否正确
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
	auto pCalcBarycentric = [](Eigen::Vector2f vA, Eigen::Vector2f vB, Eigen::Vector2f vC, Eigen::Vector2f vPoint) -> Eigen::Vector3f
	{
		Eigen::Vector3f Temp[2];
		for (int i = 2; i--; )
		{
			Temp[i].x() = vC.data()[i] - vA.data()[i];
			Temp[i].y() = vB.data()[i] - vA.data()[i];
			Temp[i].z() = vA.data()[i] - vPoint.data()[i];
		}
		Eigen::Vector3f TempVec = Temp[0].cross(Temp[1]); //u, v, 1
		return Eigen::Vector3f(1.0f - (TempVec.x() + TempVec.y()) / TempVec.z(), TempVec.y() / TempVec.z(), TempVec.x() / TempVec.z());
	};

	//8 faces plane
	m_Mesh = _loadMesh(PlaneMeshPath);
	m_pTextureBaker = _createBaker(m_Mesh);

	std::vector<Eigen::Vector2i> ResolutionList = { {512, 512}, {1, 1} };
	std::vector<int> NumWholeTexels(ResolutionList.size(), 0);
	auto& Vertices = m_Mesh.m_Vertices;
	for (auto& Face : m_Mesh.m_Faces)
	{
		{
			auto& Resolution = ResolutionList[0];
			int NumTexels = Resolution.x() * Resolution.y();
			auto TexelInfos = m_pTextureBaker->findTexelsPerFace(Face, Resolution);
			ASSERT_TRUE(!TexelInfos.empty());
			EXPECT_NEAR(TexelInfos.size(), NumTexels * (1 / 8), NumTexels * 0.1);
			NumWholeTexels[0] += TexelInfos.size();

			for (auto& Texel : TexelInfos)
			{
				Eigen::Vector2f PointUV = { Texel.TexelPos.x() / Resolution.x(), Texel.TexelPos.y() / Resolution.y() };
				Eigen::Vector2f FacesUV[3];
				for (int i = 0; i < 3; i++)
					FacesUV[i] = { Vertices[Texel.OriginFace[i]].u, Vertices[Texel.OriginFace[i]].v };

				auto Centric = pCalcBarycentric(FacesUV[0], FacesUV[1], FacesUV[2], PointUV);
				Eigen::Vector3f ExpectPos = Centric.x() * Vertices[Texel.OriginFace[0]].xyz() + Centric.y() * Vertices[Texel.OriginFace[1]].xyz() + Centric.z() * Vertices[Texel.OriginFace[2]].xyz();
				const float ErrorScope = 1.0f;
				for (int i = 0; i < 3; i++)
					ASSERT_NEAR(Texel.TexelPosInWorld.data()[i], ExpectPos.data()[i], ErrorScope);
			}
		}

		{
			auto& Resolution = ResolutionList[1];
			auto TexelInfos = m_pTextureBaker->findTexelsPerFace(Face, Resolution);
			NumWholeTexels[1] += TexelInfos.size();
		}	
	}

	//总处理纹素要接近总纹素数
	for (int i = 0; i < ResolutionList.size(); i++)
	{
		auto& Resolution = ResolutionList[i];
		int NumTexels = Resolution.x() * Resolution.y();
		EXPECT_NEAR(NumWholeTexels[i], NumTexels, NumTexels * 0.1);
	}

}

TEST_F(TestCastingTextureBaker, TestExecuteIntersection)
{

}

TEST_F(TestCastingTextureBaker, TestCalcTexelColor)
{

}

