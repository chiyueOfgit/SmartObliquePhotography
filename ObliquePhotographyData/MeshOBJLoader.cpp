#include "pch.h"
#include "MeshOBJLoader.h"
#include <regex>
#include <wrap/io_trimesh/import_obj.h>
#include <wrap/io_trimesh/io_material.h>
#include "VcgMesh.hpp"
#include "Mesh.h"

using namespace hiveObliquePhotography;

_REGISTER_EXCLUSIVE_PRODUCT(CMeshOBJLoader, OBJ_LOADER)

//*****************************************************************
//FUNCTION:
int CMeshOBJLoader::__loadDataFromFileV(const std::string& vFileName, CMesh& voMesh)
{
	//CMesh Mesh;
	CVcgMesh VcgMesh;

	auto ImportInfo = vcg::tri::io::ImporterOBJ<CVcgMesh>::Info();
	auto ErrorCode = vcg::tri::io::ImporterOBJ<CVcgMesh>::Open(VcgMesh, vFileName.c_str(), ImportInfo);
	fromVcgMesh(VcgMesh, voMesh);

	//CVcgMesh::PerFaceAttributeHandle<int> mIndHandle =
	//	vcg::tri::Allocator<CVcgMesh>::GetPerFaceAttribute<int>(VcgMesh, std::string("materialIndex"));
	//auto material_index = mIndHandle;

	CVcgMesh::PerMeshAttributeHandle<std::vector<vcg::tri::io::Material>> materialsHandle =
		vcg::tri::Allocator<CVcgMesh>::GetPerMeshAttribute<std::vector<vcg::tri::io::Material> >(VcgMesh, std::string("materialVector"));

	voMesh.setMaterial(fromVcgMaterial(materialsHandle().front()));
	return ErrorCode;
}