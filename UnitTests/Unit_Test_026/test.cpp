#include "pch.h"
#include "ArapParameterizer.h"
#include "SceneReconstructionConfig.h"
#include "ObliquePhotographyDataInterface.h"
#include "VcgMesh.hpp"
#include <igl/decimate.h>
#include <vcg/complex/algorithms/clean.h>

//测试用例列表：
//*Test_buildHalfEdgeTest: 测试建立的半边数据结构是否符合预期。


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

	CArapParameterizer* _createProduct(const hiveObliquePhotography::CMesh& vMesh)
	{
		auto pParameterization =  hiveDesignPattern::hiveCreateProduct<IMeshParameterizer>(KEYWORD::ARAP_MESH_PARAMETERIZATION, CSceneReconstructionConfig::getInstance()->getSubConfigByName("Parameterization"), vMesh);
		EXPECT_NE(pParameterization, nullptr);
		if (!pParameterization)
			std::cerr << "create baker error." << std::endl;
		return dynamic_cast<CArapParameterizer*>(pParameterization);
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

	hiveObliquePhotography::CMesh m_Mesh;
	pcl::TexMaterial m_Material;
	std::string m_MeshPath;

	CArapParameterizer* m_pMeshParameterization = nullptr;
};

TEST_F(TestArapParameterization, Test_buildHalfEdgeTest)
{
	m_pMeshParameterization->buildHalfEdge();
	std::string PerLine, PerVertexId, PerPrev, PerNext, PerConj, PerFace;
	std::vector<SHalfEdge> TestHalfEdgeTable;
	int TestHalfEdgeTableSize;
	TestHalfEdgeTableSize = 36;
	std::ifstream TestFile;
	TestFile.open(CubeTestDataPath);
	while (getline(TestFile, PerLine))
	{
		std::istringstream ContentPerLine(PerLine);
		SHalfEdge TestHalfEdge;

		ContentPerLine >> PerVertexId >> PerPrev >> PerNext >> PerConj >> PerFace;
		TestHalfEdge._VertexId = _stringToInt(PerVertexId);
		TestHalfEdge._Prev = _stringToInt(PerPrev);
		TestHalfEdge._Next = _stringToInt(PerNext);
		TestHalfEdge._Conj = _stringToInt(PerConj);
		TestHalfEdge._Face = _stringToInt(PerFace);
		TestHalfEdgeTable.emplace_back(TestHalfEdge);
	}
	TestFile.close();
	EXPECT_EQ(m_pMeshParameterization->getHalfEdgeTable().size(), TestHalfEdgeTableSize);
	for (int HalfEdgeId = 0; HalfEdgeId < TestHalfEdgeTableSize; HalfEdgeId++)
	{
		EXPECT_EQ(TestHalfEdgeTable[HalfEdgeId]._VertexId, m_pMeshParameterization->getHalfEdgeTable()[HalfEdgeId]._VertexId);
		EXPECT_EQ(TestHalfEdgeTable[HalfEdgeId]._Prev, m_pMeshParameterization->getHalfEdgeTable()[HalfEdgeId]._Prev);
		EXPECT_EQ(TestHalfEdgeTable[HalfEdgeId]._Next, m_pMeshParameterization->getHalfEdgeTable()[HalfEdgeId]._Next);
		EXPECT_EQ(TestHalfEdgeTable[HalfEdgeId]._Conj, m_pMeshParameterization->getHalfEdgeTable()[HalfEdgeId]._Conj);
		EXPECT_EQ(TestHalfEdgeTable[HalfEdgeId]._Face, m_pMeshParameterization->getHalfEdgeTable()[HalfEdgeId]._Face);
	}
}
