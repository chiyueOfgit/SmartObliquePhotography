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
	m_pTextureBaker = _createBaker(m_Mesh);

	std::vector<Eigen::Vector2i> ResolutionList = { {512, 512}, {10, 10} };
	std::vector<int> NumWholeTexels(ResolutionList.size(), 0);
	auto& Vertices = m_Mesh.m_Vertices;
	for (auto& Face : m_Mesh.m_Faces)
	{
		{
			auto& Resolution = ResolutionList[0];
			int NumTexels = Resolution.x() * Resolution.y();
			auto TexelInfos = m_pTextureBaker->findTexelsPerFace(Face, Resolution);
			ASSERT_TRUE(!TexelInfos.empty());
			EXPECT_NEAR(TexelInfos.size(), NumTexels * ((float)1 / 8), NumTexels * 0.1);
			NumWholeTexels[0] += TexelInfos.size();

			for (auto& Texel : TexelInfos)
			{
				Eigen::Vector2f PointUV = { (Texel.TexelPos.x() + 0.5f) / Resolution.x(), (Texel.TexelPos.y() + 0.5f) / Resolution.y() };
				Eigen::Vector2f FacesUV[3];
				for (int i = 0; i < 3; i++)
					FacesUV[i] = { Vertices[Texel.OriginFace[i]].u, Vertices[Texel.OriginFace[i]].v };

				Eigen::Vector2f DeltaPos = { PointUV.x() * 100.0f, -PointUV.y() * 100.0f };
				Eigen::Vector2f BeginPos{ -50.0f, 50.0f };

				Eigen::Vector3f TexelPosInPlane = { BeginPos.x() + DeltaPos.x(), 0.0f, BeginPos.y() + DeltaPos.y() };
				const float ErrorScope = 1.0f;
				for (int i = 0; i < 3; i++)
					ASSERT_NEAR(Texel.TexelPosInWorld.data()[i], TexelPosInPlane.data()[i], ErrorScope);
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

TEST_F(TestCastingTextureBaker, TestExecuteIntersection_1)
{
	auto pCloud = hiveObliquePhotography::hiveInitPointCloudScene({ TESTMODEL_DIR + std::string("/Test024_Model/TestPointCloud.ply") });
	m_pTextureBaker->setPointCloud(pCloud);
	
	STexelInfo TestTexel{ {98,98},{48.0f, 0.0f, 48.0f},m_Mesh.m_Faces[2] };
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

	STexelInfo TestTexel{ {51,51},{1.5f, 0.0f, 1.5f},m_Mesh.m_Faces[3] };
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
	auto pCloud = hiveObliquePhotography::hiveInitPointCloudScene({ TESTMODEL_DIR + std::string("/Test024_Model/TestPointCloud.ply") });
	m_pTextureBaker->bakeTexture(pCloud, { 512, 512 });
}

TEST_F(TestCastingTextureBaker, PlaneTextureBakingTest)
{
	auto PNGFilePath = TESTMODEL_DIR + std::string("Test024_Model/100RG.png");
	Eigen::Vector2i Resolution{ 10, 10 };
	auto pCloud = hiveObliquePhotography::hiveInitPointCloudScene({ TESTMODEL_DIR + std::string("Test024_Model/100RGPointCloud.ply") });
	
	auto Texture = m_pTextureBaker->bakeTexture(pCloud, Resolution);
	int Channels = 3;
	unsigned char* GtData = stbi_load(PNGFilePath.c_str(), &Resolution.x(), &Resolution.y(), &Channels, 0);
	for (int i = 0; i < Resolution.x() * Resolution.y(); i++)
	{
		Eigen::Vector3i GtColor{ GtData[i * 3], GtData[i * 3 + 1], GtData[i * 3 + 2] };
		Eigen::Vector3i Color = Texture.getColor(i / Resolution.x(), i % Resolution.x());
		EXPECT_EQ(Color, GtColor);
	}

	{
		const auto Width = Texture.getWidth();
		const auto Height = Texture.getHeight();
		const auto BytesPerPixel = 3;
		auto ResultImage = new unsigned char[Width * Height * BytesPerPixel];
		for (auto i = 0; i < Height; i++)
			for (auto k = 0; k < Width; k++)
			{
				auto Offset = (i * Width + k) * BytesPerPixel;
				ResultImage[Offset] = Texture.getColor(i, k)[0];
				ResultImage[Offset + 1] = Texture.getColor(i, k)[1];
				ResultImage[Offset + 2] = Texture.getColor(i, k)[2];
			}

		stbi_write_png("Test.png", Width, Height, BytesPerPixel, ResultImage, 0);
		stbi_image_free(ResultImage);
	}
}

//TEST_F(TestCastingTextureBaker, Generator)
//{
//	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Output(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//	//float X = -50.0f, Z = -50.0f;
//	for(int i = 0; i < 10; i++)
//	{
//		float Z = -50.0f + (2 * i + 1) * 5.0f;
//		for(int k = 0; k < 10; k++)
//		{
//			float X = -50.0f + (2 * k + 1) * 5.0f;
//			pcl::PointXYZRGBNormal TempPoint;
//			TempPoint.x = X; TempPoint.y = 1.0f; TempPoint.z = Z;
//			TempPoint.normal_x = 0.0f; TempPoint.normal_y = 1.0f; TempPoint.normal_z = 0.0f;
//			if(i%2)
//			{
//				TempPoint.r = 255; TempPoint.g = 0; TempPoint.b = 0;
//			}
//			else
//			{
//				TempPoint.r = 0; TempPoint.g = 255; TempPoint.b = 0;
//			}
//			Output->push_back(TempPoint);
//		}
//	}
//	pcl::io::savePLYFile(TESTMODEL_DIR + std::string("/Test024_Model/100RGPointCloud.ply"), *Output);
//
//	
//	Eigen::Vector2i Resolution{ 10, 10 };
//	int Channels = 3;
//	unsigned char* data = stbi_load("D://VS2019Repos//SmartObliquePhotography//UnitTests//TestData//Test024_Model//100RG.png", &Resolution.x(), &Resolution.y(), &Channels, 0);
//
//	 for (int i = 0; i < 10; i++)
//	{
//		for(int k = 0; k < 10; k++)
//		{
//			if (i % 2)
//			{
//				data[i * 10 * 3 + k * 3] = 0;
//				data[i * 10 * 3 + k * 3 + 1] = 255;
//				data[i * 10 * 3 + k * 3 + 2] = 0;
//			}			    	    
//			else		    	    
//			{			    	    
//				data[i * 10 * 3 + k * 3] = 255;
//				data[i * 10 * 3 + k * 3 + 1] = 0;
//				data[i * 10 * 3 + k * 3 + 2] = 0;
//			}
//				
//		}
//	}
//	 stbi_write_png("D://VS2019Repos//SmartObliquePhotography//UnitTests//TestData//Test024_Model//100RG.png", Resolution.x(), Resolution.y(), 3, data, 0);
//}