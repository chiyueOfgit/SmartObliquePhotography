#include "pch.h"
#include "ArapParameterization.h"

using namespace hiveObliquePhotography::SceneReconstruction;

_REGISTER_NORMAL_PRODUCT(CArapParameterization, KEYWORD::ARAP_MESH_PARAMETERIZATION)

//*****************************************************************
//FUNCTION: 
CArapParameterization::CArapParameterization()
{
	m_VertexInfoTable.resize(m_Mesh.m_Vertices.size());
}

void CArapParameterization::buildHalfEdge()
{
	
}


std::vector<int> CArapParameterization::findBoundaryPoint()
{
	return {};
}