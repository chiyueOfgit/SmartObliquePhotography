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

class TestArapParameterization : public testing::Test
{
protected:
	void SetUp() override
	{
		m_Mesh = _loadMesh(ScuMeshPath);
		m_pMeshParameterization = _createProduct(m_Mesh);
	}

	void TearDown() override
	{
		delete m_pMeshParameterization;
	}

	hiveObliquePhotography::CMesh _loadMesh(const std::string& vPath)
	{
		pcl::TextureMesh TexMesh;
		pcl::io::loadOBJFile(vPath, TexMesh);
		m_Material = TexMesh.tex_materials[0];
		hiveObliquePhotography::CMesh Mesh(TexMesh);
		bool EmptyFlag = Mesh.m_Vertices.empty() || Mesh.m_Faces.empty();
		EXPECT_FALSE(EmptyFlag);
		if (EmptyFlag)
			std::cerr << "mesh load error." << std::endl;
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

					FixedLine += Vp + "/" + Vt + "/" + "1" + " ";
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