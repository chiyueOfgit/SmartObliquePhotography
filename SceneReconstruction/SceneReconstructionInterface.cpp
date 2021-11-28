#include "pch.h"
#include "SceneReconstructionInterface.h"
#include "SceneReconstructionConfig.h"
#include "PoissonSurfaceReconstructor.h"
#include "CollapseBasedSimplification.h"
#include "BasicMeshSuture.h"
#include "ArapParameterizer.h"
#include "RayCastingBaker.h"
#include "MeshSutureManager.h"

#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>

using namespace hiveObliquePhotography::SceneReconstruction;

//*****************************************************************
//FUNCTION: 
void hiveObliquePhotography::SceneReconstruction::hiveSurfaceReconstruction(PointCloud_t::Ptr vSceneCloud, CMesh& voMesh)
{
	auto pPoisson = hiveDesignPattern::hiveCreateProduct<ISurfaceReconstructor>(KEYWORD::POISSON_RECONSTRUCTOR, CSceneReconstructionConfig::getInstance()->getSubConfigByName("PoissonReconstruction"), vSceneCloud);
	_ASSERTE(pPoisson);
	pPoisson->constructSurface(voMesh);
	auto pSimplifier = hiveDesignPattern::hiveCreateProduct<IMeshSimplifacation>(KEYWORD::COLLAPSE_BASED_SIMPLIFICATION, CSceneReconstructionConfig::getInstance()->getSubConfigByName("PoissonReconstruction"), voMesh);
	_ASSERTE(pSimplifier);
	//voMesh = pSimplifier->simplifyMesh();
}

void hiveObliquePhotography::SceneReconstruction::hiveMeshSimplication(CMesh& vioMesh)
{
	auto pSimplifier = hiveDesignPattern::hiveCreateProduct<IMeshSimplifacation>(KEYWORD::COLLAPSE_BASED_SIMPLIFICATION, CSceneReconstructionConfig::getInstance()->getSubConfigByName("PoissonReconstruction"), vioMesh);
	_ASSERTE(pSimplifier);
	vioMesh = pSimplifier->simplifyMesh();
}

//*****************************************************************
//FUNCTION: 
void hiveObliquePhotography::SceneReconstruction::hiveSutureMesh(CMesh& vioMeshOne, CMesh& vioMeshTwo)
{
	auto pSuturator = hiveDesignPattern::hiveCreateProduct<CBasicMeshSuture>(KEYWORD::BASIC_MESH_SUTURE, CSceneReconstructionConfig::getInstance()->getSubConfigByName("BasicSuture"), vioMeshOne, vioMeshTwo);
	pSuturator->sutureMeshesV();
	pSuturator->dumpMeshes(vioMeshOne, vioMeshTwo);
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::SceneReconstruction::hiveMeshParameterization(CMesh& vioMesh)
{
	if (vioMesh.m_Vertices.empty())
	{
		hiveEventLogger::hiveOutputEvent("Parameterization: Size of Vertices in mesh: 0.");
		return false;
	}
		
	auto pParameterizater = hiveDesignPattern::hiveCreateProduct<CArapParameterizer>(KEYWORD::ARAP_MESH_PARAMETERIZATION, CSceneReconstructionConfig::getInstance()->getSubConfigByName("Parameterization"), vioMesh);
	//_ASSERTE(pParameterizater);

	if (pParameterizater == nullptr)
	{
		hiveEventLogger::hiveOutputEvent("Parameterization: Create product error.");
		return false;
	}

	Eigen::MatrixXd UV;
	if (!(pParameterizater->execute(UV)))
	{
		hiveEventLogger::hiveOutputEvent("Parameterization: Failed to solve the equation.");
		return false;
	}

	_ASSERTE(UV.rows() == vioMesh.m_Vertices.size());
	for (int i = 0; i < UV.rows(); i++)
	{
		vioMesh.m_Vertices[i].u = UV.row(i).x();
		vioMesh.m_Vertices[i].v = UV.row(i).y();
	}
	return true;
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::SceneReconstruction::hiveBakeColorTexture(const CMesh& vMesh, PointCloud_t::Ptr vSceneCloud, CImage<std::array<int, 3>>& voImage)
{
	auto pBaker = hiveDesignPattern::hiveCreateProduct<CRayCastingBaker>(KEYWORD::RAYCASTING_TEXTUREBAKER, CSceneReconstructionConfig::getInstance()->getSubConfigByName("RayCasting"), vMesh);
	voImage = pBaker->bakeTexture(vSceneCloud);
	if (voImage.getHeight() <= 0 || voImage.getWidth() <= 0)
		return false;
	else
		return true;
}

//*****************************************************************
//FUNCTION: 
pcl::TextureMesh hiveObliquePhotography::SceneReconstruction::hiveTestCMesh(const std::string& vPath)
{
	pcl::TextureMesh Mesh;
	pcl::io::loadOBJFile(vPath, Mesh);
	pcl::TextureMesh Mesh2;
	pcl::io::loadPolygonFileOBJ(vPath, Mesh2);
	Mesh2.tex_materials = Mesh.tex_materials;
	auto Material = Mesh2.tex_materials[0];

	CMesh M(Mesh2);

	return M.toTexMesh(Material);
}

//*****************************************************************
//FUNCTION:
bool hiveObliquePhotography::SceneReconstruction::hiveGetSceneReconstructionConfig(CSceneReconstructionConfig*& voConfig)
{
	auto pConfig = CSceneReconstructionConfig::getInstance();
	_ASSERTE(pConfig);
	if (pConfig)
	{
		voConfig = CSceneReconstructionConfig::getInstance();
		return true;
	}
	else
	{
		voConfig = nullptr;
		return false;
	}
}

//*****************************************************************
//FUNCTION:
bool hiveObliquePhotography::SceneReconstruction::hiveRecordTileInfo(const PointCloud_t::Ptr vTileCloud, const std::string vName)
{
	auto pManager = CMeshSutureManager::getInstance();
	_ASSERTE(pManager);
	if (pManager)
	{
		pManager->calTileInfo(vTileCloud, vName);
		return true;
	}
	else
	{
		return false;
	}
}

//*****************************************************************
//FUNCTION:
bool hiveObliquePhotography::SceneReconstruction::hiveGetSutureSequence(std::vector<pair<std::string, std::string>>& voSutureNames)
{
	auto pManager = CMeshSutureManager::getInstance();
	_ASSERTE(pManager);
	if (pManager && pManager->calSutureSequence(voSutureNames))
		return true;

	return false;
}
