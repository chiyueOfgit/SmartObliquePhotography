#include "pch.h"
#include "MeshParameterizer.h"

using namespace hiveObliquePhotography::SceneReconstruction;

//*****************************************************************
//FUNCTION:
bool IMeshParameterizer::onProductCreatedV(const hiveConfig::CHiveConfig* vConfig, const CMesh& vMesh)
{
	m_pConfig = vConfig;
	m_Mesh = vMesh;
	return true;
}
