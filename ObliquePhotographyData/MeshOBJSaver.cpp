#include "pch.h"
#include "MeshOBJSaver.h"
#include "Mesh.h"
#include <regex>

using namespace hiveObliquePhotography;

_REGISTER_EXCLUSIVE_PRODUCT(CMeshOBJSaver, OBJ_SAVER);

//*****************************************************************
//FUNCTION: 
void CMeshOBJSaver::saveDataToFileV(const CMesh& vMesh, const std::string& vFilePath)
{
	auto Name = vFilePath.substr(vFilePath.find_last_of("/") + 1, vFilePath.find(".") - vFilePath.find_last_of("/") - 1);
	auto PathName = vFilePath.substr(0, vFilePath.find("."));

	std::ofstream ObjFile(vFilePath);
	std::string ObjLines;
	std::string Comments;
	std::string Pos;
	std::string Tex;
	std::string Normal;
	std::string FaceStr;
	if (ObjFile.is_open())
	{
		Comments += "# Vertices: " + std::to_string(vMesh.m_Vertices.size()) + "\n";
		Comments += "# Faces: " + std::to_string(vMesh.m_Faces.size()) + "\n";
		for (auto& Vertex : vMesh.m_Vertices)
		{
			Pos += _FORMAT_STR3("v %1% %2% %3%\n", std::to_string(Vertex.x), std::to_string(Vertex.y), std::to_string(Vertex.z));
			Tex += _FORMAT_STR2("vt %1% %2%\n", std::to_string(Vertex.u), std::to_string(Vertex.v));
			Normal += _FORMAT_STR3("vn %1% %2% %3%\n", std::to_string(Vertex.nx), std::to_string(Vertex.ny), std::to_string(Vertex.nz));
		}

		for (auto& Face : vMesh.m_Faces)
		{
			FaceStr += "f ";
			for (int i = 0; i < 3; i++)
			{
				auto Index = std::to_string(Face[i] + 1);
				FaceStr += _FORMAT_STR3("%1%/%2%/%3% ", Index, Index, Index);
			}
			FaceStr += "\n";
		}

		ObjLines += Comments;
		ObjLines += "mtllib " + Name + ".mtl\n" + "\n";
		ObjLines += Pos + "\n" + "usemtl " + Name + "\n" + Tex + "\n" + Normal + "\n";
		ObjLines += FaceStr;

		ObjFile << ObjLines;
		ObjFile.close();
	}

	std::ofstream MtlFile(PathName + ".mtl");
	std::string MtlLines;
	if (MtlFile.is_open())
	{
		MtlLines += "newmtl " + Name + "\n";
		MtlLines += "Ka 0.2 0.2 0.2\n";
		MtlLines += "Kd 1.0 1.0 1.0\n";
		MtlLines += "Ks 0.2 0.2 0.2\n";
		MtlLines += "d 1.0\n";
		MtlLines += "Ns 32.0\n";
		MtlLines += "Ni 1.0\n";
		MtlLines += "illum 2\n";
		MtlLines += "map_Kd \n";

		MtlFile << MtlLines;
		MtlFile.close();
	}
}