#pragma once
#include "Mesh.h"

namespace hiveObliquePhotography::SceneReconstruction
{
	void intersectMeshAndPlane(const Eigen::Vector4f& vPlane, CMesh& vioMesh, std::vector<SVertex>& voIntersectionPoints, std::vector<int>& voDissociatedIndices);
	std::pair<std::vector<SVertex>, std::vector<int>> intersectMeshAndPlane(const Eigen::Vector4f& vPlane, CMesh& vioMesh);
}