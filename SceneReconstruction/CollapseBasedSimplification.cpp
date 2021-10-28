#include "pch.h"
#include "CollapseBasedSimplification.h"
#include "VcgMesh.hpp"
#include <igl/decimate.h>
#include <vcg/complex/algorithms/clean.h>

using namespace hiveObliquePhotography::SceneReconstruction;

_REGISTER_NORMAL_PRODUCT(CCollapseBasedSimplification, KEYWORD::COLLAPSE_BASED_SIMPLIFICATION)

//*****************************************************************
//FUNCTION: 
void CCollapseBasedSimplification::__toManifold(CMesh& vioMesh)         
{
	_ASSERTE(!(vioMesh.m_Vertices.empty() || vioMesh.m_Faces.empty()));
	CVcgMesh VcgMesh;
	toVcgMesh(vioMesh, VcgMesh);                                   
	vcg::tri::Clean<CVcgMesh>::SplitNonManifoldVertex(VcgMesh, 0.1);
	hiveObliquePhotography::fromVcgMesh(VcgMesh, vioMesh);
}

//*****************************************************************
//FUNCTION: 
hiveObliquePhotography::CMesh CCollapseBasedSimplification::simplifyMesh()     
{
	__toManifold(m_Mesh);

	Eigen::MatrixXd V = m_Mesh.getVerticesMatrix();
	Eigen::MatrixXi F = m_Mesh.getFacesMatrix();
	Eigen::VectorXi J, I;
	Eigen::MatrixXd OV;
	Eigen::MatrixXi OF;
	bool a = igl::decimate(V, F, 0.9 * F.rows(), OV, OF, J, I);
	hiveObliquePhotography::CMesh SimplifiedMesh(OV, OF);
	return SimplifiedMesh;
}