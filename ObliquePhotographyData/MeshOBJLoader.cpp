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
		std::vector<Eigen::Vector3f> Normals;
		std::vector<Eigen::Vector2f> TexCoords;

		std::string NumberRegex = "(-?\\d+.?\\d+e?-?\\d*)";
		std::string SpaceRegex = " +";

		std::regex ThreeNumRegex(NumberRegex + SpaceRegex + NumberRegex + SpaceRegex + NumberRegex);
		std::regex TwoNumRegex(NumberRegex + SpaceRegex + NumberRegex);
		std::regex FaceRegex("(\\d*)/(\\d*)/(\\d*)");
		std::smatch Result;

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
		do
		{
			if (Line[0] == 'f')
			{
				std::vector<uint32_t> Face;
				for (auto Begin = Line.cbegin(); std::regex_search(Begin, Line.cend(), Result, FaceRegex); Begin = Result.suffix().first)
				{
					_ASSERTE(Result.size() == 4);
					auto VertexId = std::stoi(Result[1]) - 1;
					auto TexId = Result[2] != "" ? std::stoi(Result[2]) - 1 : -1;
					auto NormalId = Result[3] != "" ? std::stoi(Result[3]) - 1 : -1;

					//处理位置
					Face.push_back(VertexId);	//0为全部匹配

					//处理纹理
					if (TexId != -1)
					{
						Mesh.m_Vertices[VertexId].u = TexCoords[TexId].x();
						Mesh.m_Vertices[VertexId].v = TexCoords[TexId].y();
					}

					//处理法线
					if (NormalId != -1)
					{
						Mesh.m_Vertices[VertexId].nx = Normals[NormalId].x();
						Mesh.m_Vertices[VertexId].ny = Normals[NormalId].y();
						Mesh.m_Vertices[VertexId].nz = Normals[NormalId].z();
					}
				}

				Mesh.m_Faces.push_back({ Face[0], Face[1], Face[2] });
			}
			else if (Line.substr(0, 2) == "v ")
			{
				std::regex_search(Line, Result, ThreeNumRegex);
				_ASSERTE(Result.size() == 4);
				hiveObliquePhotography::SVertex Vertex;
				Vertex.x = std::stod(Result[1]);
				Vertex.y = std::stod(Result[2]);
				Vertex.z = std::stod(Result[3]);
				Mesh.m_Vertices.push_back(Vertex);
			}
			else if (Line.substr(0, 2) == "vt")
			{
				std::regex_search(Line, Result, TwoNumRegex);
				_ASSERTE(Result.size() == 3);
				Eigen::Vector2f TexCoord;
				TexCoord.x() = std::stod(Result[1]);
				TexCoord.y() = std::stod(Result[2]);
				TexCoords.push_back(TexCoord);
			}
			else if (Line.substr(0, 2) == "vn")
			{
				std::regex_search(Line, Result, ThreeNumRegex);
				_ASSERTE(Result.size() == 4);
				Eigen::Vector3f Normal;
				Normal.x() = std::stod(Result[1]);
				Normal.y() = std::stod(Result[2]);
				Normal.z() = std::stod(Result[3]);
				Normals.push_back(Normal);
			}
		} while (std::getline(ObjFileIn, Line));

		ObjFileIn.close();
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
				std::smatch Result;
				std::regex VertexRegex(NumberRegex + SpaceRegex + NumberRegex + SpaceRegex + NumberRegex);
				std::regex_search(Line, Result, VertexRegex);
				_ASSERTE(Result.size() == 4);
				Material.tex_Ka = { std::stof(Result[1]), std::stof(Result[2]), std::stof(Result[3]) };
			}
			if (Line.find("Kd ") != std::string::npos && Line.find("map_Kd") == std::string::npos)
			{
				std::smatch Result;
				std::regex VertexRegex(NumberRegex + SpaceRegex + NumberRegex + SpaceRegex + NumberRegex);
				std::regex_search(Line, Result, VertexRegex);
				_ASSERTE(Result.size() == 4);
				Material.tex_Kd = { std::stof(Result[1]), std::stof(Result[2]), std::stof(Result[3]) };
			}
			if (Line.find("Ks ") != std::string::npos)
			{
				std::smatch Result;
				std::regex VertexRegex(NumberRegex + SpaceRegex + NumberRegex + SpaceRegex + NumberRegex);
				std::regex_search(Line, Result, VertexRegex);
				_ASSERTE(Result.size() == 4);
				Material.tex_Ks = { std::stof(Result[1]), std::stof(Result[2]), std::stof(Result[3]) };
			}
		}

	return Material;
}