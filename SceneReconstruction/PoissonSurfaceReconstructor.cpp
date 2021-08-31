#include "pch.h"
#include "PoissonSurfaceReconstructor.h"
#include <pcl/surface/poisson.h>

using namespace hiveObliquePhotography::SceneReconstruction;

_REGISTER_NORMAL_PRODUCT(CPoissonSurfaceReconstructor, KEYWORD::POISSON_RECONSTRUCTOR)

//*****************************************************************
//FUNCTION: 
void CPoissonSurfaceReconstructor::constructSurface(pcl::PolygonMesh& voMesh)
{
	pcl::Poisson<pcl::PointNormal> PoissonConstructor;
	PoissonConstructor.setDepth(9);
	PoissonConstructor.setInputCloud(m_pSceneCloud);
	PoissonConstructor.performReconstruction(voMesh);
}
