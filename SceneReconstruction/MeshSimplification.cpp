#include "pch.h"
#include "MeshSimplification.h"

using namespace hiveObliquePhotography::SceneReconstruction;

//*****************************************************************
//FUNCTION:
bool IMeshSimplifacation::onProductCreatedV(const hiveConfig::CHiveConfig* vConfig, const CMesh& vMesh)
{
	m_pConfig = vConfig;
	m_Mesh = vMesh;
	return true;
}
