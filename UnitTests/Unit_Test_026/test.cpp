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

class TestArapParameterization : public testing::Test
{
protected:
	void SetUp() override
	{
		m_MeshPath = PlaneMeshPath;
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

	hiveObliquePhotography::CMesh m_Mesh;
	pcl::TexMaterial m_Material;
	std::string m_MeshPath;

	CArapParameterizer* m_pMeshParameterization = nullptr;
};
