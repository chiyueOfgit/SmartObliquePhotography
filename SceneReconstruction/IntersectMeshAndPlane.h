#pragma once
#include "Mesh.h"

namespace hiveObliquePhotography::SceneReconstruction
{
	void intersectMeshAndPlane(CMesh& vioMesh, const Eigen::Vector4f& vPlane, std::vector<SVertex>& voIntersectionPoints, std::vector<int>& voDissociatedIndices);
}