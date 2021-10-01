#include "pch.h"

#include "SceneReconstructionConfig.h"
#include "MeshPlaneIntersection.h"

#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <fstream>
#include <regex>

using namespace hiveObliquePhotography::SceneReconstruction;

const auto SimpleMeshPath = TESTMODEL_DIR + std::string("Test027_Model/SimpleMesh.obj");
const auto SceneMeshPath = TESTMODEL_DIR + std::string("Test027_Model/005006.obj");

class TestMeshPlaneIntersection : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void LoadMesh(const std::string& vPath)
	{
		m_Mesh = _loadObj(vPath);
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
	hiveObliquePhotography::CMesh m_Mesh;
};

TEST_F(TestMeshPlaneIntersection, Test_NoIntersection)
{
	LoadMesh(SimpleMeshPath);

	Eigen::Vector4f Plane(1.0, 0.0, 0.0, 3.0);
	hiveObliquePhotography::CMesh DissociatedMesh;
	std::vector<hiveObliquePhotography::SVertex> IntersectionPoints;
	CMeshPlaneIntersection MeshPlaneIntersection;
	MeshPlaneIntersection.execute(m_Mesh, Plane);
	MeshPlaneIntersection.dumpDissociatedMesh(DissociatedMesh);
	MeshPlaneIntersection.dumpIntersectionPoints(IntersectionPoints);
	EXPECT_EQ(DissociatedMesh.m_Vertices.size(), 0);
	EXPECT_EQ(IntersectionPoints.size(), 0);
}

TEST_F(TestMeshPlaneIntersection, Test_SeveralIntersections)
{
	LoadMesh(SimpleMeshPath);
	hiveObliquePhotography::CMesh DissociatedMesh;
	std::vector<hiveObliquePhotography::SVertex> IntersectionPoints;
	CMeshPlaneIntersection MeshPlaneIntersection;

	Eigen::Vector4f PlaneWithOneIntersection(1.0, 0.0, 0.0, 2.0);
	MeshPlaneIntersection.execute(m_Mesh, PlaneWithOneIntersection);
	MeshPlaneIntersection.dumpDissociatedMesh(DissociatedMesh);
	MeshPlaneIntersection.dumpIntersectionPoints(IntersectionPoints);
	EXPECT_EQ(DissociatedMesh.m_Vertices.size(), 0);
	EXPECT_EQ(IntersectionPoints.size(), 1);

	Eigen::Vector4f PlaneWithSeveralIntersections(1.0, 0.0, 0.0, 1.5);
	MeshPlaneIntersection.execute(m_Mesh, PlaneWithSeveralIntersections);
	MeshPlaneIntersection.dumpDissociatedMesh(DissociatedMesh);
	MeshPlaneIntersection.dumpIntersectionPoints(IntersectionPoints);
	EXPECT_EQ(DissociatedMesh.m_Vertices.size(), 1);
	EXPECT_EQ(IntersectionPoints.size(), 2);

	Eigen::Vector4f PlaneWithSeveralIntersectionsOverlapWithOrigin(1.0, 0.0, 0.0, 0.0);
	MeshPlaneIntersection.execute(m_Mesh, PlaneWithSeveralIntersectionsOverlapWithOrigin);
	MeshPlaneIntersection.dumpDissociatedMesh(DissociatedMesh);
	MeshPlaneIntersection.dumpIntersectionPoints(IntersectionPoints);
	EXPECT_EQ(DissociatedMesh.m_Vertices.size(), 3);
	EXPECT_EQ(IntersectionPoints.size(), 3);

}

TEST_F(TestMeshPlaneIntersection, Test_TerrainModel)
{
	LoadMesh(SceneMeshPath);
	hiveObliquePhotography::CMesh DissociatedMesh;
	std::vector<hiveObliquePhotography::SVertex> IntersectionPoints;
	CMeshPlaneIntersection MeshPlaneIntersection;

	Eigen::Vector4f Plane(1.0, 0.0, 0.0, -132.0);
	MeshPlaneIntersection.execute(m_Mesh, Plane);
	MeshPlaneIntersection.dumpDissociatedMesh(DissociatedMesh);
	MeshPlaneIntersection.dumpIntersectionPoints(IntersectionPoints);
	EXPECT_EQ(DissociatedMesh.m_Vertices.size(), 153);
	EXPECT_EQ(IntersectionPoints.size(), 433);
}