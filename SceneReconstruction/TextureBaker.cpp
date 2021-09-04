#include "pch.h"
#include "TextureBaker.h"

using namespace hiveObliquePhotography::SceneReconstruction;

//*****************************************************************
//FUNCTION: 
bool ITextureBaker::onProductCreatedV(const hiveConfig::CHiveConfig* vConfig, const CMesh& vMesh)
{
	m_pConfig = vConfig;
	m_Mesh = vMesh;
	return true;
}
