#include "pch.h"
#include "MeshOBJLoader.h"
#include "Mesh.h"
#include <regex>

using namespace hiveObliquePhotography;

_REGISTER_EXCLUSIVE_PRODUCT(CMeshOBJLoader, OBJ_LOADER)

//*****************************************************************
//FUNCTION:
int CMeshOBJLoader::__loadDataFromFileV(const std::string& vFileName, CMesh& voMesh)
{
	hiveObliquePhotography::CMesh Mesh;
	pcl::TexMaterial Material;

	auto FolderPath = vFileName.substr(0, vFileName.find_last_of("/") + 1);
	std::ifstream ObjFileIn(vFileName);
	if (ObjFileIn.is_open())
	{
		std::string Line;
		while (std::getline(ObjFileIn, Line) && Line[0] != 'v')
		{
			if (Line[0] == '#')
				continue;
			if (Line.find("mtllib") != std::string::npos)	// load mtl
			{
				std::string MtlPath;
				if (Line.find("./") != std::string::npos)
					MtlPath = FolderPath + Line.substr(Line.find('/') + 1, Line.length());
				else
					MtlPath = FolderPath + Line.substr(Line.find(' ') + 1, Line.length());
				Material = __loadMaterialFromFile(MtlPath);
			}
		}
		if (Material.tex_name != "")
			Mesh.setMaterial(Material);
		voMesh = Mesh;
		return 0;
	}
	else
		return -1;
}

pcl::TexMaterial CMeshOBJLoader::__loadMaterialFromFile(const std::string& vFileName)
{
	pcl::TexMaterial Material;

	std::string Line;

	std::string NumberRegex = "(-?\\d+.?\\d+e?-?\\d*)";
	std::string SpaceRegex = " +";

	std::regex ThreeNumRegex(NumberRegex + SpaceRegex + NumberRegex + SpaceRegex + NumberRegex);
	std::smatch Result;

	std::ifstream MtlFileIn(vFileName);
	if (MtlFileIn.is_open())
		while (std::getline(MtlFileIn, Line))
		{
			if (Line[0] == '#')
				continue;
			if (Line.find("newmtl ") != std::string::npos)
				Material.tex_name = Line.substr(Line.find(' ') + 1, Line.length());
			if (Line.find("map_Kd") != std::string::npos)
			{
				std::string MapPath;
				auto FolderPath = vFileName.substr(0, vFileName.find_last_of("/") + 1);
				if (Line.find("./") != std::string::npos)
					MapPath = FolderPath + Line.substr(Line.find('/') + 1, Line.length());
				else
					MapPath = FolderPath + Line.substr(Line.find(' ') + 1, Line.length());
				Material.tex_file = MapPath;
			}
			if (Line.find("Ns ") != std::string::npos)
				Material.tex_Ns = std::stof(Line.substr(Line.find(' ') + 1, Line.length()));
			if (Line.find("d ") != std::string::npos && Line.find("Kd") == std::string::npos)
				Material.tex_d = std::stof(Line.substr(Line.find(' ') + 1, Line.length()));
			if (Line.find("illum") != std::string::npos)
				Material.tex_illum = std::stoi(Line.substr(Line.find(' ') + 1, Line.length()));
			if (Line.find("Ka ") != std::string::npos)
			{
				std::regex_search(Line, Result, ThreeNumRegex);
				_ASSERTE(Result.size() == 4);
				Material.tex_Ka = { std::stof(Result[1]), std::stof(Result[2]), std::stof(Result[3]) };
			}
			if (Line.find("Kd ") != std::string::npos && Line.find("map_Kd") == std::string::npos)
			{
				std::regex_search(Line, Result, ThreeNumRegex);
				_ASSERTE(Result.size() == 4);
				Material.tex_Kd = { std::stof(Result[1]), std::stof(Result[2]), std::stof(Result[3]) };
			}
			if (Line.find("Ks ") != std::string::npos)
			{
				std::regex_search(Line, Result, ThreeNumRegex);
				_ASSERTE(Result.size() == 4);
				Material.tex_Ks = { std::stof(Result[1]), std::stof(Result[2]), std::stof(Result[3]) };
			}
		}

	return Material;
}