#include "pch.h"
#include "PoissonSurfaceReconstructor.h"
#include <pcl/surface/poisson.h>

using namespace hiveObliquePhotography::SceneReconstruction;

_REGISTER_NORMAL_PRODUCT(CPoissonSurfaceReconstructor, KEYWORD::POISSON_RECONSTRUCTOR)

//*****************************************************************
//FUNCTION: 
void CPoissonSurfaceReconstructor::constructSurface(pcl::PolygonMesh& voMesh)
{
	pcl::Poisson<pcl::PointNormal> Poisson;
	Poisson.setDepth(m_pConfig->getAttribute<int>(KEYWORD::OCTREE_DEPTH).value());
	Poisson.setInputCloud(m_pSceneCloud);
	Poisson.performReconstruction(voMesh);
}
