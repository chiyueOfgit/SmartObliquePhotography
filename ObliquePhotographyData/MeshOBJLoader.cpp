#include "pch.h"
#include "MeshOBJLoader.h"
#include <wrap/io_trimesh/import_obj.h>
#include <wrap/io_trimesh/io_material.h>
#include "VcgMesh.hpp"

using namespace hiveObliquePhotography;

_REGISTER_EXCLUSIVE_PRODUCT(CMeshOBJLoader, OBJ_LOADER)

//*****************************************************************
//FUNCTION:
int CMeshOBJLoader::__loadDataFromFileV(const std::string& vFileName, CMesh& voMesh)
{
	auto Split = vFileName.find_last_of('/') + 1;
	auto FolderPath = vFileName.substr(0, Split);
	auto PureName = vFileName.substr(Split);
	
	WCHAR CurrentDirectory[MAX_PATH] = { 0 };
	GetCurrentDirectory(MAX_PATH, CurrentDirectory);
	SetCurrentDirectoryA(FolderPath.c_str());

	CVcgMesh VcgMesh;
	auto ImportInfo = vcg::tri::io::ImporterOBJ<CVcgMesh>::Info();
	auto ErrorCode = vcg::tri::io::ImporterOBJ<CVcgMesh>::Open(VcgMesh, PureName.c_str(), ImportInfo);
	fromVcgMesh(VcgMesh, voMesh);
	SetCurrentDirectory(CurrentDirectory);

	CVcgMesh::PerMeshAttributeHandle<std::vector<vcg::tri::io::Material>> materialsHandle =
		vcg::tri::Allocator<CVcgMesh>::GetPerMeshAttribute<std::vector<vcg::tri::io::Material> >(VcgMesh, std::string("materialVector"));
	if (!materialsHandle().empty())
	{
		auto Material = fromVcgMaterial(materialsHandle().front());
		if (hiveUtility::hiveLocateFile(Material.tex_file).empty())
			Material.tex_file.insert(0, FolderPath);

		voMesh.setMaterial(Material);
	}
	
	return ErrorCode;
}