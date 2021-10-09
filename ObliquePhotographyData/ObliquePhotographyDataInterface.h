#pragma once
#include "ObliquePhotographyDataExport.h"
#include "Mesh.h"

namespace hiveObliquePhotography
{
	OPDATA_DECLSPEC std::vector<PointCloud_t::Ptr> hiveInitPointCloudScene(const std::vector<std::string>& vFileNameSet);
	OPDATA_DECLSPEC bool hiveSavePointCloudScene(PointCloud_t::Ptr vPointCloud, const std::string& vFileName);
	OPDATA_DECLSPEC void hiveLoadMeshModel(CMesh& voMesh, const std::string& vFileName);
	OPDATA_DECLSPEC void hiveSaveMeshModel(const CMesh& vMesh, const std::string& vFileName);
}