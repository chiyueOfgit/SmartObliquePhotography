#include "pch.h"
#include "MeshSuture.h"

using namespace hiveObliquePhotography::SceneReconstruction;

//*****************************************************************
//FUNCTION: 
bool IMeshSuture::onProductCreatedV(const hiveConfig::CHiveConfig* vConfig, const CMesh& vMeshLHS, const CMesh& vMeshRHS)
{
	m_pConfig = vConfig;
	m_MeshLHS = vMeshLHS;
	m_MeshRHS = vMeshRHS;
	return true;
}