#include "pch.h"
#include "MeshSuture.h"

using namespace hiveObliquePhotography::SceneReconstruction;

//*****************************************************************
//FUNCTION: 
bool IMeshSuture::onProductCreatedV(const hiveConfig::CHiveConfig* vConfig, const CMesh& vLhs, const CMesh& vRhs)
{
	m_pConfig = vConfig;
	m_LhsMesh = vLhs;
	m_RhsMesh = vRhs;
	return true;
}
