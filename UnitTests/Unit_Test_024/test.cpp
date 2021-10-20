#include "pch.h"
#include "RayCastingBaker.h"
#include "SceneReconstructionConfig.h"

#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include "ObliquePhotographyDataInterface.h"

#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

//测试用例列表：
//  * bakeTexture: 测试在简单场景下生成纹理贴图的正确性
//
//  * findTexelsPerFace: 测试每三角面片所覆盖的纹素及对应世界坐标是否正确
// 
//  * executeIntersection: 测试执行光线投射后与点云面片相交结果是否正确
//  * TestExecuteIntersection_1: 测试点云点在面片法线一侧的相交情况
//  * TestExecuteIntersection_2: 测试点云点在面片两侧的相交情况
// 
//  * calcTexelColor: 
// 

using namespace hiveObliquePhotography::SceneReconstruction;

const auto PlaneMeshPath = TESTMODEL_DIR + std::string("/Test024_Model/Plane/Plane100.obj");
const auto CloudPath = TESTMODEL_DIR + std::string("/Test024_Model/100RGPointCloud.ply");

class TestCastingTextureBaker : public testing::Test
{
protected:
	void SetUp() override
	{
		m_Mesh = _loadMesh(PlaneMeshPath);

		m_TileSet = hiveObliquePhotography::hiveInitPointCloudScene({ CloudPath });

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

	void _saveTexture(const std::string& vPath, const hiveObliquePhotography::CImage<std::array<int, 3>>& vTexture, bool vIsReverse)
	{
		const auto Width = vTexture.getWidth();
		const auto Height = vTexture.getHeight();
		const auto BytesPerPixel = 3;
		auto ResultImage = new unsigned char[Width * Height * BytesPerPixel];
		for (auto i = 0; i < Height; i++)
			for (auto k = 0; k < Width; k++)
			{
				auto I = i;
				if (vIsReverse)
					I = Height - 1 - I;
				auto Offset = (I * Width + k) * BytesPerPixel;
				ResultImage[Offset] = vTexture.getColor(i, k)[0];
				ResultImage[Offset + 1] = vTexture.getColor(i, k)[1];
				ResultImage[Offset + 2] = vTexture.getColor(i, k)[2];
			}

		stbi_write_png(vPath.c_str(), Width, Height, BytesPerPixel, ResultImage, 0);
		stbi_image_free(ResultImage);
	}

	hiveObliquePhotography::CMesh m_Mesh;
	std::vector<PointCloud_t::Ptr> m_TileSet;

	CRayCastingBaker* m_pTextureBaker = nullptr;
};

//TEST_F(TestCastingTextureBaker, TestBakeEffect)
//{
//	const auto ModelPath = TESTMODEL_DIR + std::string("/Test024_Model/scu/Tile16.obj");
//	const auto CloudPath = TESTMODEL_DIR + std::string("/Test024_Model/scu/Tile16.pcd");
//
//	m_Mesh = _loadMesh(ModelPath);
//	m_pCloud = hiveObliquePhotography::hiveInitPointCloudScene({ CloudPath });
//	m_pTextureBaker = _createBaker(m_Mesh);
//
//	Eigen::Vector2i Resolution = { 512, 512 };
//	auto Texture = m_pTextureBaker->bakeTexture(m_pCloud, Resolution);
//	_saveTexture("Test.png", Texture, true);
//}

TEST_F(TestCastingTextureBaker, TestFindTexelsPerFace)
{
	//8 faces plane
	m_Mesh = _loadMesh(PlaneMeshPath);
	m_pTextureBaker = _createBaker(m_Mesh);

	std::vector<Eigen::Vector2i> ResolutionList = { {512, 512}, {10, 10} };
	std::vector<int> NumWholeTexels(ResolutionList.size(), 0);
	auto& Vertices = m_Mesh.m_Vertices;
	for (auto& Face : m_Mesh.m_Faces)
	{
		{
			auto& Resolution = ResolutionList[0];
			int NumTexels = Resolution.x() * Resolution.y();
			auto TexelInfos = m_pTextureBaker->findSamplesPerFace(Face, Resolution);
			ASSERT_TRUE(!TexelInfos.empty());
			EXPECT_NEAR(TexelInfos.size(), NumTexels * ((float)1 / 8), NumTexels * 0.1);
			NumWholeTexels[0] += TexelInfos.size();

			for (auto& Texel : TexelInfos)
			{
				Eigen::Vector2f PointUV = { (Texel.TexelCoord.x() + 0.5f) / Resolution.x(), (Texel.TexelCoord.y() + 0.5f) / Resolution.y() };
				Eigen::Vector2f DeltaPos = { PointUV.x() * 100.0f, -PointUV.y() * 100.0f };
				Eigen::Vector2f BeginPos{ -50.0f, 50.0f };
				Eigen::Vector3f TexelPosInPlane = { BeginPos.x() + DeltaPos.x(), 0.0f, BeginPos.y() + DeltaPos.y() };
                const float ErrorScope = 50.0f;
				for (auto& Ray : Texel.RaySet)
				{
                   for (int i = 0; i < 3; i++)
					  EXPECT_NEAR(Ray.Origin.data()[i], TexelPosInPlane.data()[i], ErrorScope);
				}
			}
		}

		{
			auto& Resolution = ResolutionList[1];
			auto TexelInfos = m_pTextureBaker->findSamplesPerFace(Face, Resolution);
			NumWholeTexels[1] += TexelInfos.size();
		}	
	}

	//总处理纹素要接近总纹素数
	for (int i = 0; i < ResolutionList.size(); i++)
	{
		auto& Resolution = ResolutionList[i];
		int NumTexels = Resolution.x() * Resolution.y();
		EXPECT_NEAR(NumWholeTexels[i], NumTexels, NumTexels * 0.2);
	}
}

TEST_F(TestCastingTextureBaker, TestExecuteIntersection_1)
{
	auto pTileSet = hiveObliquePhotography::hiveInitPointCloudScene({ TESTMODEL_DIR + std::string("/Test024_Model/TestPointCloud.ply") });
	m_pTextureBaker->setPointCloud(pTileSet.front());
	
	SRay TestRay{ {48.0f, 2.0f, 48.0f},{0.0f, 1.0f, 0.0f}};
	auto CandidateSet = m_pTextureBaker->executeIntersection(TestRay);
	ASSERT_EQ(CandidateSet.size(), 3);
	sort(CandidateSet.begin(), CandidateSet.end(), [](SCandidateInfo& vA, SCandidateInfo& vB) {return vA.SurfelIndex < vB.SurfelIndex; });
	Eigen::Vector3f IntersectionOne{ 48.0f, 2.0f, 48.0f };
	Eigen::Vector3f IntersectionTwo{ 48.0f, 3.0f, 48.0f };
	EXPECT_EQ(CandidateSet[0].SurfelIndex, 0);
	EXPECT_EQ(CandidateSet[0].Intersection, IntersectionOne);
	EXPECT_EQ(CandidateSet[1].SurfelIndex, 1);
	EXPECT_EQ(CandidateSet[1].Intersection, IntersectionTwo);
	EXPECT_EQ(CandidateSet[2].SurfelIndex, 2);
	EXPECT_EQ(CandidateSet[2].Intersection, IntersectionOne);
}

TEST_F(TestCastingTextureBaker, TestExecuteIntersection_2)
{
	auto pTileSet = hiveObliquePhotography::hiveInitPointCloudScene({ TESTMODEL_DIR + std::string("/Test024_Model/TestPointCloud.ply") });
	m_pTextureBaker->setPointCloud(pTileSet.front());

	SRay TestRay{ {1.5f, 0.0f, 1.5f},{0.0f, 1.0f, 0.0f} };
	auto CandidateSet = m_pTextureBaker->executeIntersection(TestRay);
	ASSERT_EQ(CandidateSet.size(), 2);
	sort(CandidateSet.begin(), CandidateSet.end(), [](SCandidateInfo& vA, SCandidateInfo& vB) {return vA.SurfelIndex < vB.SurfelIndex; });
	Eigen::Vector3f IntersectionOne{ 1.5f, 2.0f, 1.5f };
	Eigen::Vector3f IntersectionTwo{ 1.5f, -2.0f, 1.5f };
	EXPECT_EQ(CandidateSet[0].SurfelIndex, 3);
	EXPECT_EQ(CandidateSet[0].Intersection, IntersectionOne);
	EXPECT_EQ(CandidateSet[1].SurfelIndex, 4);
	EXPECT_EQ(CandidateSet[1].Intersection, IntersectionTwo);
}

TEST_F(TestCastingTextureBaker, TestCalcTexelColor)
{
	auto pTileSet = hiveObliquePhotography::hiveInitPointCloudScene({ TESTMODEL_DIR + std::string("/Test024_Model/TestPointCloud.ply") });
	m_pTextureBaker->bakeTexture(pTileSet.front(), { 512, 512 });
}

TEST_F(TestCastingTextureBaker, PlaneTextureBakingTest)
{
	auto PNGFilePath = TESTMODEL_DIR + std::string("Test024_Model/100RG.png");
	Eigen::Vector2i Resolution{ 10, 10 };
	auto pTileSet = hiveObliquePhotography::hiveInitPointCloudScene({ TESTMODEL_DIR + std::string("Test024_Model/100RGPointCloud.ply") });
	
	auto Texture = m_pTextureBaker->bakeTexture(pTileSet.front(), Resolution);
	int Channels = 3;
	unsigned char* GtData = stbi_load(PNGFilePath.c_str(), &Resolution.x(), &Resolution.y(), &Channels, 0);
	for (int i = 0; i < Resolution.x() * Resolution.y(); i++)
	{
		std::array<int, 3> GtColor{ GtData[i * 3], GtData[i * 3 + 1], GtData[i * 3 + 2] };
		auto Color = Texture.getColor(i / Resolution.x(), i % Resolution.x());
		EXPECT_EQ(Color, GtColor);
	}

	_saveTexture("Test.png", Texture, false);
}