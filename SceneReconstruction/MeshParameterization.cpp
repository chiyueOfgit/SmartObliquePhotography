#include "pch.h"
#include "MeshParameterization.h"

using namespace hiveObliquePhotography::SceneReconstruction;

//*****************************************************************
//FUNCTION:
bool IMeshParameterization::onProductCreatedV(const hiveConfig::CHiveConfig* vConfig, const CMesh& vMesh)
{
	m_pConfig = vConfig;
	m_Mesh = vMesh;
	return true;
}
