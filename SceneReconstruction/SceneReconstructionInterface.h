#pragma once
#include "SceneReconstructionExport.h"

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		RECONSTRUCTION_DECLSPEC void hiveSurfaceReconstruction(PointCloud_t::Ptr vSceneCloud, pcl::PolygonMesh& voMesh);
	}
}