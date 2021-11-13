#pragma once
#include "SceneReconstructionExport.h"
#include "SceneReconstructionConfig.h"
#include "Mesh.h"
#include "Image.h"

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		RECONSTRUCTION_DECLSPEC void hiveSurfaceReconstruction(PointCloud_t::Ptr vSceneCloud, CMesh& voMesh);
		RECONSTRUCTION_DECLSPEC void hiveMeshSimplication(CMesh& vioMesh);
		RECONSTRUCTION_DECLSPEC bool hiveMeshParameterization(CMesh& vioMesh);
		RECONSTRUCTION_DECLSPEC void hiveSutureMesh(CMesh& vioMeshOne, CMesh& vioMeshTwo);
		RECONSTRUCTION_DECLSPEC bool hiveBakeColorTexture(const CMesh& vMesh, PointCloud_t::Ptr vSceneCloud, CImage<std::array<int, 3>>& voImage);
		RECONSTRUCTION_DECLSPEC pcl::TextureMesh hiveTestCMesh(const std::string& vPath);
		RECONSTRUCTION_DECLSPEC bool hiveGetSceneReconstructionConfig(CSceneReconstructionConfig*& voConfig);
		RECONSTRUCTION_DECLSPEC bool hiveRecordTileInfo(const PointCloud_t::Ptr vTileCloud, const std::string vName);
		RECONSTRUCTION_DECLSPEC bool hiveGetSutureSequence(std::vector<std::pair<std::string, std::string>> voSutureNames);
	}
}