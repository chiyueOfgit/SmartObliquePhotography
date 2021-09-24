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

	std::ifstream ObjFileIn(vFileName);
	if (ObjFileIn.is_open())
	{
		std::string Line;
		std::vector<Eigen::Vector3f> Normals;
		std::vector<Eigen::Vector2f> TexCoords;
		std::string NumberRegex = "(-?\\d+.?\\d+e?-?\\d*)";
		std::string SpaceRegex = " +";
		while (std::getline(ObjFileIn, Line))
		{
			if (Line[0] == '#')
				continue;
			if (Line.find("mtllib") != std::string::npos)
			{
			}
			if (Line.substr(0, 2) == "v ")
			{
				std::smatch Result;
				std::regex VertexRegex(NumberRegex + SpaceRegex + NumberRegex + SpaceRegex + NumberRegex);
				std::regex_search(Line, Result, VertexRegex);
				_ASSERTE(Result.size() == 4);
				hiveObliquePhotography::SVertex Vertex;
				Vertex.x = std::stod(Result[1]);
				Vertex.y = std::stod(Result[2]);
				Vertex.z = std::stod(Result[3]);
				Mesh.m_Vertices.push_back(Vertex);
			}
			if (Line.substr(0, 2) == "vt")
			{
				std::smatch Result;
				std::regex VertexRegex(NumberRegex + SpaceRegex + NumberRegex);
				std::regex_search(Line, Result, VertexRegex);
				_ASSERTE(Result.size() == 3);
				Eigen::Vector2f TexCoord;
				TexCoord.x() = std::stod(Result[1]);
				TexCoord.y() = std::stod(Result[2]);
				TexCoords.push_back(TexCoord);
			}
			if (Line.substr(0, 2) == "vn")
			{
				std::smatch Result;
				std::regex VertexRegex(NumberRegex + SpaceRegex + NumberRegex + SpaceRegex + NumberRegex);
				std::regex_search(Line, Result, VertexRegex);
				_ASSERTE(Result.size() == 4);
				Eigen::Vector3f Normal;
				Normal.x() = std::stod(Result[1]);
				Normal.y() = std::stod(Result[2]);
				Normal.z() = std::stod(Result[3]);
				Normals.push_back(Normal);
			}
			if (Line[0] == 'f')
			{
				std::smatch Result;
				std::regex FaceRegex("(\\d+)/(\\d+)/(\\d+)");
				std::vector<uint32_t> Face;
				for (auto Begin = Line.cbegin(); std::regex_search(Begin, Line.cend(), Result, FaceRegex); Begin = Result.suffix().first)
				{
					_ASSERTE(Result.size() == 4);
					auto VertexId = std::stoi(Result[1]) - 1;
					auto TexId = std::stoi(Result[2]) - 1;
					auto NormalId = std::stoi(Result[3]) - 1;

					//处理位置
					Face.push_back(VertexId);	//0为全部匹配

					//处理纹理
					Mesh.m_Vertices[VertexId].u = TexCoords[TexId].x();
					Mesh.m_Vertices[VertexId].v = TexCoords[TexId].y();

					//处理法线
					Mesh.m_Vertices[VertexId].nx = Normals[NormalId].x();
					Mesh.m_Vertices[VertexId].ny = Normals[NormalId].y();
					Mesh.m_Vertices[VertexId].nz = Normals[NormalId].z();
				}
				Mesh.m_Faces.push_back({ Face[0], Face[1], Face[2] });
			}
		}

		ObjFileIn.close();
		voMesh = Mesh;
		return 0;
	}
	else
		return -1;
}