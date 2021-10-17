#include "pch.h"
#include "SceneReconstructionInterface.h"
#include "SceneReconstructionConfig.h"
#include "PoissonSurfaceReconstructor.h"
#include "CollapseBasedSimplification.h"
#include "BasicMeshSuture.h"
#include "ArapParameterizer.h"
#include "RayCastingBaker.h"

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
}

void hiveObliquePhotography::SceneReconstruction::hiveMeshSimplication(CMesh& vioMesh)
{
	auto pSimplifier = hiveDesignPattern::hiveCreateProduct<IMeshSimplifacation>(KEYWORD::COLLAPSE_BASED_SIMPLIFICATION, CSceneReconstructionConfig::getInstance()->getSubConfigByName("PoissonReconstruction"), vioMesh);
	_ASSERTE(pSimplifier);
	vioMesh = pSimplifier->simplifyMesh();
}

//*****************************************************************
//FUNCTION: 
RECONSTRUCTION_DECLSPEC void hiveObliquePhotography::SceneReconstruction::hiveSutureMesh(CMesh& vioMeshOne, CMesh& vioMeshTwo, PointCloud_t::Ptr vCloudOne, PointCloud_t::Ptr vCloudTwo)
{
	auto pSuturator = hiveDesignPattern::hiveCreateProduct<CBasicMeshSuture>(KEYWORD::BASIC_MESH_SUTURE, CSceneReconstructionConfig::getInstance()->getSubConfigByName("BasicSuture"), vioMeshOne, vioMeshTwo);
	pSuturator->setCloud4SegmentPlane(vCloudOne, vCloudTwo);
	pSuturator->sutureMeshesV();
	pSuturator->dumpMeshes(vioMeshOne, vioMeshTwo);
}

//*****************************************************************
//FUNCTION: 
void hiveObliquePhotography::SceneReconstruction::hiveMeshParameterization(CMesh& vioMesh)
{
	_ASSERTE(!vioMesh.m_Vertices.empty());
	auto pParameterizater = hiveDesignPattern::hiveCreateProduct<CArapParameterizer>(KEYWORD::ARAP_MESH_PARAMETERIZATION, CSceneReconstructionConfig::getInstance()->getSubConfigByName("Parameterization"), vioMesh);
	_ASSERTE(pParameterizater);

	auto UV = pParameterizater->execute();
	_ASSERTE(UV.rows() == vioMesh.m_Vertices.size());
	for (int i = 0; i < UV.rows(); i++)
	{
		vioMesh.m_Vertices[i].u = UV.row(i).x();
		vioMesh.m_Vertices[i].v = UV.row(i).y();
	}
}

//*****************************************************************
//FUNCTION: 
hiveObliquePhotography::CImage<std::array<int, 3>> hiveObliquePhotography::SceneReconstruction::hiveBakeColorTexture(const CMesh& vMesh, PointCloud_t::Ptr vSceneCloud, Eigen::Vector2i vResolution)
{
	auto pBaker = hiveDesignPattern::hiveCreateProduct<CRayCastingBaker>(KEYWORD::RAYCASTING_TEXTUREBAKER, CSceneReconstructionConfig::getInstance()->getSubConfigByName("RayCasting"), vMesh);
	return pBaker->bakeTexture(vSceneCloud, vResolution);
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
