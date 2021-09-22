#include "pch.h"
#include "ArapParameterization.h"
#include "SceneReconstructionConfig.h"

#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <fstream>
#include <regex>

//测试用例列表：
//   * findBoundaryPoint: 测试在简单场景下寻找边界点正确性。


using namespace hiveObliquePhotography::SceneReconstruction;

const auto PlaneMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Plane/Plane100.obj");
const auto ScuMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Scu/Tile16.obj");
const auto StoneMeshPath = TESTMODEL_DIR + std::string("/Test026_Model/Others/LI_Rock_Pavers.obj");

class TestArapParameterization : public testing::Test
{
protected:
	void SetUp() override
	{
		m_Mesh = _loadObj(PlaneMeshPath);
		m_pMeshParameterization = _createProduct(m_Mesh);
	}

	void TearDown() override
	{
		delete m_pMeshParameterization;
	}

	hiveObliquePhotography::CMesh _loadMesh(const std::string& vPath)
	{
		pcl::PolygonMesh TexMesh;
		pcl::io::loadOBJFile(vPath, TexMesh);
		//m_Material = TexMesh.tex_materials[0];
		hiveObliquePhotography::CMesh Mesh(TexMesh);
		bool EmptyFlag = Mesh.m_Vertices.empty() || Mesh.m_Faces.empty();
		EXPECT_FALSE(EmptyFlag);
		if (EmptyFlag)
			std::cerr << "mesh load error." << std::endl;
		return Mesh;
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
					std::regex VertexRegex("(-?\\d+.?\\d+) +(-?\\d+.?\\d+) +(-?\\d+.?\\d+)");
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
					std::regex VertexRegex("(-?\\d+.?\\d+) +(-?\\d+.?\\d+)");
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
					std::regex VertexRegex("(-?\\d+.?\\d+) +(-?\\d+.?\\d+) +(-?\\d+.?\\d+)");
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

	CArapParameterization* _createProduct(const hiveObliquePhotography::CMesh& vMesh)
	{
		auto pParameterization =  hiveDesignPattern::hiveCreateProduct<IMeshParameterization>(KEYWORD::ARAP_MESH_PARAMETERIZATION, CSceneReconstructionConfig::getInstance()->getSubConfigByName("RayCasting"), vMesh);
		EXPECT_NE(pParameterization, nullptr);
		if (!pParameterization)
			std::cerr << "create baker error." << std::endl;
		return dynamic_cast<CArapParameterization*>(pParameterization);
	}

	hiveObliquePhotography::CMesh m_Mesh;
	pcl::TexMaterial m_Material;

	CArapParameterization* m_pMeshParameterization = nullptr;
};


TEST_F(TestArapParameterization, TestfindBoundaryPoint)
{
	//int Sum = 0;
	//m_pMeshParameterization->buildHalfEdge();
	//auto PointSet = m_pMeshParameterization->findBoundaryPoint();
	//for(auto Flag: PointSet)
	//{
	//	if(Flag)
	//	{
	//		Sum++;
	//	}
	//}
	//EXPECT_EQ(Sum, 8);
	//int i = 0;
	//EXPECT_EQ(PointSet.size(), 9);

	auto UV = m_pMeshParameterization->execute();
	EXPECT_EQ(UV.rows(), m_Mesh.m_Vertices.size());
	for (int Row = 0; Row < UV.rows(); Row++)
	{
		m_Mesh.m_Vertices[Row].u = UV.row(Row)(0);
		m_Mesh.m_Vertices[Row].v = UV.row(Row)(1);
	}

	std::string ObjName = "Plane.obj";
	pcl::io::saveOBJFile(ObjName, m_Mesh.toTexMesh(m_Material));
	std::ifstream ObjFileIn(ObjName);
	if (ObjFileIn.is_open())
	{
		std::string Line;
		std::string FileLines;
		while (std::getline(ObjFileIn, Line))
		{
			if (Line[0] == 'f')
			{
				std::string FixedLine("f ");

				std::smatch Result;
				std::regex FaceRegex("\\d+/\\d+/\\d+");
				for (auto Begin = Line.cbegin(); std::regex_search(Begin, Line.cend(), Result, FaceRegex); Begin = Result.suffix().first)
				{
					auto FaceStr = Result[0].str();
					auto FirstPartition = FaceStr.find('/');
					auto SecondPartition = FaceStr.find_last_of('/');
					auto Vp = FaceStr.substr(0, FirstPartition);
					auto Vn = FaceStr.substr(FirstPartition + 1, SecondPartition - FirstPartition - 1);
					auto Vt = FaceStr.substr(SecondPartition + 1, FaceStr.length());

					FixedLine += Vp + "/" + Vt + "/" + Vn + " ";
				}
				FileLines += FixedLine;
			}
			else
				FileLines += Line;
			FileLines += "\n";
		}

		ObjFileIn.close();

		std::ofstream ObjFileOut(ObjName);
		ObjFileOut << FileLines;
	}
}