#pragma once
#include "SceneReconstructionExport.h"
#include "Mesh.h"

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		RECONSTRUCTION_DECLSPEC void hiveSurfaceReconstruction(PointCloud_t::Ptr vSceneCloud, CMesh& voMesh);
		RECONSTRUCTION_DECLSPEC pcl::TextureMesh hiveTestCMesh(const std::string& vPath);
	}
}