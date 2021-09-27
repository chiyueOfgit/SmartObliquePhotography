#include "pch.h"
#include "ArapParameterizer.h"
#include "SceneReconstructionConfig.h"

#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <fstream>
#include <regex>

//测试用例列表：
//   * findBoundaryPoint: 测试在简单场景下寻找边界点正确性。


using namespace hiveObliquePhotography::SceneReconstruction;

const auto PlaneMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Plane/Plane.obj");
const auto TileMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Tile_low/005-004.obj");
const auto StoneMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Others/LI_Rock_Pavers.obj");
const auto MountainMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Others/mountain.obj");
const auto ScuMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Scu/Tile16.obj");
const auto PyramidMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Pyramid.obj");

class TestArapParameterization : public testing::Test
{
protected:
	void SetUp() override
	{
		m_MeshPath = ScuMeshPath;
		m_Mesh = _loadObj(m_MeshPath);
		m_pMeshParameterization = _createProduct(m_Mesh);
	}

	void TearDown() override
	{
		delete m_pMeshParameterization;
	}

	hiveObliquePhotography::CMesh _loadObj(const std::string& vPath)
	{
		hiveObliquePhotography::CMesh Mesh;

		std::ifstream ObjFileIn(vPath);
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
					Mesh.m_Faces.emplace_back(Face[0], Face[1], Face[2]);
				}
			}

			ObjFileIn.close();
		}

		return Mesh;
	}

	void _saveObj(const std::string& vPath, const hiveObliquePhotography::CMesh& vMesh)
	{
		ofstream ObjFile(vPath);
		std::string FileLines;
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

			auto Name = vPath.substr(0, vPath.find("."));
			FileLines += Comments;
			FileLines += "mtllib " + Name + ".mtl\n" + "\n";
			FileLines += Pos + "\n" + "usemtl " + Name + "\n" + Tex + "\n" + Normal + "\n";
			FileLines += FaceStr;
		}
		ObjFile << FileLines;
		ObjFile.close();
	}

	CArapParameterizer* _createProduct(const hiveObliquePhotography::CMesh& vMesh)
	{
		auto pParameterization =  hiveDesignPattern::hiveCreateProduct<IMeshParameterizer>(KEYWORD::ARAP_MESH_PARAMETERIZATION, CSceneReconstructionConfig::getInstance()->getSubConfigByName("Parameterization"), vMesh);
		EXPECT_NE(pParameterization, nullptr);
		if (!pParameterization)
			std::cerr << "create baker error." << std::endl;
		return dynamic_cast<CArapParameterizer*>(pParameterization);
	}

	hiveObliquePhotography::CMesh m_Mesh;
	pcl::TexMaterial m_Material;
	std::string m_MeshPath;

	CArapParameterizer* m_pMeshParameterization = nullptr;
};


TEST_F(TestArapParameterization, TestfindBoundaryPoint)
{
	m_pMeshParameterization->setPath4Boundary(m_MeshPath);
	auto UV = m_pMeshParameterization->execute();
	EXPECT_EQ(UV.rows(), m_Mesh.m_Vertices.size());
	for (int Row = 0; Row < UV.rows(); Row++)
	{
		m_Mesh.m_Vertices[Row].u = UV.row(Row).x();
		m_Mesh.m_Vertices[Row].v = UV.row(Row).y();
	}

	std::string ObjName = "Plane.obj";
	_saveObj(ObjName, m_Mesh);
}