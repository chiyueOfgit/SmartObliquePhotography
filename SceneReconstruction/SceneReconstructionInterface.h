#pragma once
#include "SceneReconstructionExport.h"
#include "Mesh.h"
#include "Image.h"

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		RECONSTRUCTION_DECLSPEC void hiveSurfaceReconstruction(PointCloud_t::Ptr vSceneCloud, CMesh& voMesh);
		RECONSTRUCTION_DECLSPEC pcl::TextureMesh hiveTestCMesh(const std::string& vPath);
		RECONSTRUCTION_DECLSPEC void hiveMeshParameterization(CMesh& vioMesh, const std::string& vPath);
		RECONSTRUCTION_DECLSPEC CImage<std::array<int, 3>> hiveBakeColorTexture(const CMesh& vMesh, PointCloud_t::Ptr vSceneCloud, Eigen::Vector2i vResolution);
	}
}