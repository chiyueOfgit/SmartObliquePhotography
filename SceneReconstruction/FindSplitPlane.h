#pragma once
#include "Mesh.h"

namespace hiveObliquePhotography::SceneReconstruction
{
	Eigen::Vector4f findSplitPlane(const CMesh& vLhs, const CMesh& vRhs);
}
