#include "pch.h"
#include "SceneReconstructionInterface.h"
#include "PoissonSurfaceReconstructor.h"

using namespace hiveObliquePhotography::SceneReconstruction;

//*****************************************************************
//FUNCTION: 
void hiveObliquePhotography::SceneReconstruction::hiveSurfaceReconstruction(PointCloud_t::Ptr vSceneCloud, pcl::PolygonMesh& voMesh)
{
	auto pPoisson = hiveDesignPattern::hiveCreateProduct<ISurfaceReconstructor>(KEYWORD::POISSON_RECONSTRUCTOR, nullptr, vSceneCloud);
	_ASSERTE(pPoisson);
	pPoisson->constructSurface(voMesh);
}
