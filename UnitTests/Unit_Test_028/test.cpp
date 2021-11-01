#include "pch.h"
#include <vcg/complex/algorithms/clean.h>
#include "SceneReconstructionConfig.h"
#include "ObliquePhotographyDataInterface.h"
#include "VcgMesh.hpp"

//测试用例列表：
//  * Test_NoHoles: 判断网格是否有洞
//	* Test_NoFlips: 判断网格是否有面片翻转（如果某面片的法向量与其它面片不一样，则发生翻转）
//	* Test_NoDuplicates: 判断网格是否有面片重复
//	* Test_SelfIntersections: 判断网格是否有面片重叠

using namespace hiveObliquePhotography::SceneReconstruction;

const auto LhsMeshPath = TESTMODEL_DIR + std::string("Test028_Model/006006.obj");
const auto RhsMeshPath = TESTMODEL_DIR + std::string("Test028_Model/005006.obj");

class TestMeshLegality : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void LoadMesh(const std::string& vPath)
	{
		hiveObliquePhotography::hiveLoadMeshModel(m_Mesh, vPath);
		toVcgMesh(m_Mesh, m_VcgMesh);
	}

	hiveObliquePhotography::CMesh m_Mesh;
	hiveObliquePhotography::CVcgMesh m_VcgMesh;
};

TEST_F(TestMeshLegality, Test_NoHoles)
{
	LoadMesh(LhsMeshPath);
	int HolesCount = vcg::tri::Clean<hiveObliquePhotography::CVcgMesh>::CountHoles(m_VcgMesh);
	EXPECT_EQ(HolesCount, 0);
}

TEST_F(TestMeshLegality, Test_NoFlips)
{
	LoadMesh(LhsMeshPath);
	int FlipsCount = vcg::tri::Clean<hiveObliquePhotography::CVcgMesh>::RemoveFaceFoldByFlip(m_VcgMesh);
	EXPECT_EQ(FlipsCount, 0);
}

TEST_F(TestMeshLegality, Test_NoDuplicates)
{
	LoadMesh(LhsMeshPath);
	int OverlapsCount = vcg::tri::Clean<hiveObliquePhotography::CVcgMesh>::RemoveDuplicateFace(m_VcgMesh);
	EXPECT_EQ(OverlapsCount, 0);
}

TEST_F(TestMeshLegality, Test_SelfIntersections)
{
	LoadMesh(LhsMeshPath);
	std::vector<hiveObliquePhotography::SVcgFace*> IntersectionFaces;
	vcg::tri::Clean<hiveObliquePhotography::CVcgMesh>::SelfIntersections(m_VcgMesh, IntersectionFaces);
	int SelfIntersectionsCount = IntersectionFaces.size();
	EXPECT_EQ(SelfIntersectionsCount, 0);
}