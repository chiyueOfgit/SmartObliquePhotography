#include "pch.h"
#include "MeshOBJSaver.h"
#include "Mesh.h"


using namespace hiveObliquePhotography;

_REGISTER_EXCLUSIVE_PRODUCT(CMeshOBJSaver, OBJ_SAVER);

//*****************************************************************
//FUNCTION: 
void CMeshOBJSaver::saveDataToFileV(const CMesh& vMesh, const std::string& vFilePath)
{
	//vFilePath: Path / Name . Suffix
	//       (last of /)  (last of .)
	std::string_view Name(vFilePath.begin() + vFilePath.find_last_of('/') + 1, vFilePath.begin() + vFilePath.find_last_of('.'));
	std::string_view PathAndName(vFilePath.begin(), vFilePath.begin() + vFilePath.find_last_of('.'));
	
	std::ofstream Out(vFilePath);
	if (Out.is_open())
	{
		Out << std::format(
			"# Vertices: {}\n"
			"# Faces: {}\n"
			"mtllib {}.mtl\n\n",
			vMesh.m_Vertices.size(), vMesh.m_Faces.size(), Name
		);

		for (const auto& Vertex : vMesh.m_Vertices)
			Out << std::format("v {} {} {}\n", Vertex.x, Vertex.y, Vertex.z);
		Out << '\n';
		for (const auto& Vertex : vMesh.m_Vertices)
			Out << std::format("vt {} {}\n", Vertex.u, Vertex.v);
		Out << '\n';
		for (const auto& Vertex : vMesh.m_Vertices)
			Out << std::format("vn {} {} {}\n", Vertex.nx, Vertex.ny, Vertex.nz);
		Out << '\n';
		
		Out << std::format("usemtl {}\n", vMesh.getMaterialName());
		for (const auto& Face : vMesh.m_Faces)
			Out << std::format("f {0}/{0}/{0} {1}/{1}/{1} {2}/{2}/{2}\n", Face[0] + 1, Face[1] + 1, Face[2] + 1);

		Out.close();
	}

	vMesh.saveMaterial(std::string(PathAndName) + ".mtl");
}