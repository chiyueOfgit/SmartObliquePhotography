#include "pch.h"
#include "SceneReconstructionInterface.h"
#include "PoissonSurfaceReconstructor.h"
#include "SceneReconstructionConfig.h"

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

