#include "pch.h"

#include "SceneReconstructionConfig.h"
#include "MeshPlaneIntersection.h"
#include "ObliquePhotographyDataInterface.h"

using namespace hiveObliquePhotography::SceneReconstruction;

class TestMeshPlaneIntersection : public testing::Test
{
protected:
	void SetUp() override
	{
	}

	void LoadMesh(const std::string& vPath)
	{
		hiveObliquePhotography::hiveLoadMeshModel(m_Mesh, vPath);
	}

	hiveObliquePhotography::CMesh m_Mesh;
};

TEST_F(TestMeshPlaneIntersection, Test_NoIntersection)
{
	
}