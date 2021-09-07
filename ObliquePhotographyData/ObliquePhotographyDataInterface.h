#pragma once
#include "ObliquePhotographyDataExport.h"
#include "Mesh.h"

namespace hiveObliquePhotography
{
	OPDATA_DECLSPEC PointCloud_t::Ptr hiveInitPointCloudScene(const std::vector<std::string>& vFileNameSet);
	OPDATA_DECLSPEC bool hiveSavePointCloudScene(PointCloud_t& vPointCloud, std::string vFileName);
	OPDATA_DECLSPEC void hiveLoadMeshModel(hiveObliquePhotography::CMesh& voMesh, std::string vFileName);
}