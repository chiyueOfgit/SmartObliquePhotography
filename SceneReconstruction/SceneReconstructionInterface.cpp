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
pcl::TextureMesh hiveObliquePhotography::SceneReconstruction::hiveTestMesh()
{
	const std::string Path = "../Models/Tile_obj/Tile_078-082_low.obj";
	pcl::TextureMesh Mesh;
	pcl::io::loadOBJFile(Path, Mesh);
	pcl::TextureMesh Mesh2;
	pcl::io::loadPolygonFileOBJ(Path, Mesh2);
	Mesh2.tex_materials = Mesh.tex_materials;

	CMesh M(Mesh2);
	return Mesh2;
}

