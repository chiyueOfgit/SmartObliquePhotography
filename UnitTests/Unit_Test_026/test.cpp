#include "pch.h"
#include "ArapParameterizer.h"
#include "SceneReconstructionConfig.h"
#include "ObliquePhotographyDataInterface.h"
#include "VcgMesh.hpp"
#include <igl/decimate.h>
#include <vcg/complex/algorithms/clean.h>

//≤‚ ‘”√¿˝¡–±Ì£∫
//


using namespace hiveObliquePhotography::SceneReconstruction;

const auto PlaneMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Plane/Plane.obj");
const auto TileMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Tile_low/005-004.obj");
const auto StoneMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Others/LI_Rock_Pavers.obj");
const auto MountainMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Others/mountain.obj");
const auto ScuMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Scu/Tile16.obj");
const auto PyramidMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Pyramid.obj");
const auto PoissonMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Tile1.obj"); 
const auto CubeMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Cube.obj");
const auto CubeTestDataPath = TESTMODEL_DIR + std::string("/Test026_Model/CubeTestData.txt");

class TestArapParameterization : public testing::Test
{
protected:
	void SetUp() override
	{
		m_MeshPath = CubeMeshPath;
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
	int _stringToInt(const std::string vStringData)
	{
		std::stringstream StringToInt;
		int IntData;
		IntData = 0;
		StringToInt << vStringData;
		StringToInt >> IntData;
		return IntData;
	}

	void _testHalfEdgeTable()
	{
		std::string ContentPerLine, s1, s2, s3, s4, s5;
		//std::vector<std::vector<int>> VertexInfoTable;
		std::vector<SHalfEdge> TestHalfEdgeTable;
		TestHalfEdgeTable.resize(12 * 3);
		std::ifstream inf;
		inf.open(CubeMeshPath);

		while (getline(inf, ContentPerLine))
		{
			std::istringstream in(ContentPerLine);
			float VertexId = 0;
			int vertexNum, edgeNum, FaceId, HalfEdgeId;
			vertexNum = 0;
			edgeNum = 0;
			FaceId = 0;
			HalfEdgeId = 0;

			SHalfEdge TestHalfEdge;
			in >> s1 >> s2 >> s3 >> s4 >> s5;
			//TestHalfEdge._Face = FaceId;
			TestHalfEdge._VertexId = _stringToInt(s1);
			TestHalfEdge._Prev = _stringToInt(s2);
			TestHalfEdge._Next = _stringToInt(s3);
			TestHalfEdge._Conj = _stringToInt(s4);
			TestHalfEdge._Face = _stringToInt(s5);
			std::cout << TestHalfEdge._VertexId << " ";
			std::cout << TestHalfEdge._Prev << " ";
			std::cout << TestHalfEdge._Next << " ";
			std::cout << TestHalfEdge._Conj << " ";
			std::cout << TestHalfEdge._Face << std::endl;
			FaceId++;
			TestHalfEdgeTable.push_back(TestHalfEdge);
			//while (in >> s1 >> s2 >> s3 >> s4 >> s5) {
			//	SHalfEdge TestHalfEdge;
			//	//TestHalfEdge._Face = FaceId;
			//	TestHalfEdge._VertexId = _stringToInt(s1);
			//	TestHalfEdge._Prev = _stringToInt(s2);
			//	TestHalfEdge._Next = _stringToInt(s3);
			//	TestHalfEdge._Conj = _stringToInt(s4);
			//	TestHalfEdge._Face = _stringToInt(s5);
			//	std::cout << TestHalfEdge._VertexId << " ";
			//	std::cout << TestHalfEdge._Prev << " ";
			//	std::cout << TestHalfEdge._Next << " ";
			//	std::cout << TestHalfEdge._Conj << " ";
			//	std::cout << TestHalfEdge._Face << " ";
			//	FaceId++;
			//	TestHalfEdgeTable.push_back(TestHalfEdge);
			//}
		}

		inf.close();
		/*EXPECT_EQ(m_HalfEdgeTable.size(), m_Mesh.m_Faces.size()*3);
		for (int HalfEdgeId = 0; HalfEdgeId < m_HalfEdgeTable.size(); HalfEdgeId++)
		{
			EXPECT_EQ(1,1);
		}
		std::vector<std::vector<int>> m_VertexInfoTable;
		m_VertexInfoTable.resize(8);
		for (size_t VertexId = 0; VertexId < 8; VertexId++)
		{
			std::vector<int>
		}*/
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

TEST_F(TestArapParameterization, Test_buildHalfEdgeTest)
{
	m_pMeshParameterization->buildHalfEdge();
	_testHalfEdgeTable();
}
