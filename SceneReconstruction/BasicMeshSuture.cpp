#include "pch.h"
#include"BasicMeshSuture.h"

using namespace hiveObliquePhotography::SceneReconstruction;

_REGISTER_NORMAL_PRODUCT(CBasicMeshSuture, KEYWORD::BASIC_MESH_SUTURE)

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::sutureMeshes()
{
	
}


//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::__connectVerticesWithMesh(CMesh& vioMesh, std::vector<int>& vDissociatedIndices, std::vector<SVertex>& vPublicVertices)
{
	_ASSERTE(!vDissociatedIndices.empty() && !vPublicVertices.empty());

	std::vector<int> PublicIndices;
	for (int i = 0; i < vPublicVertices.size(); i++)
	{
		vioMesh.m_Vertices.push_back(vPublicVertices[i]);
		PublicIndices.push_back(i + vioMesh.m_Vertices.size());
	}

	auto ConnectionFaceSet = __genConnectionFace(vDissociatedIndices.size(), PublicIndices.size(), true);	// order is heuristic

	for (int Offset = vDissociatedIndices.size(); auto& Face : ConnectionFaceSet)
	{
		SFace FaceWithMeshIndex;
		for (int i = 0; i < 3; i++)
			FaceWithMeshIndex[i] = Face[i] < Offset ? vDissociatedIndices[Face[i]] : PublicIndices[Face[i] - Offset];
		vioMesh.m_Faces.push_back(FaceWithMeshIndex);
	}
}

//*****************************************************************
//FUNCTION: 
std::vector<hiveObliquePhotography::SFace> CBasicMeshSuture::__genConnectionFace(int vNumLeft, int vNumRight, bool vDefaultOrder)
{
	using namespace hiveObliquePhotography;
	std::vector<SFace> ConnectionFaceSet;

	int NumLess = vNumLeft < vNumRight ? vNumLeft : vNumRight;
	int NumMore = vNumLeft < vNumRight ? vNumRight : vNumLeft;
	float ContainRate = float(NumMore) / NumLess;

	int LessOffset = NumLess == vNumLeft ? 0 : vNumLeft;
	int MoreOffset = NumMore == vNumRight ? vNumLeft : 0;
	IndexType LessCursor = LessOffset, MoreCursor = MoreOffset;
	IndexType LessEnd = NumLess + LessOffset, MoreEnd = NumMore + MoreOffset;

	auto genFixLessFace = [&](IndexType vLess, IndexType& vMore)
	{
		SFace Face;
		if (vDefaultOrder)
			Face = { vLess, vMore, vMore + 1 };
		else
			Face = { vLess, vMore + 1, vMore };
		ConnectionFaceSet.push_back(Face);
		vMore++;
	};
	auto genFixMoreFace = [&](IndexType& vLess, IndexType vMore)
	{
		SFace Face;
		if (vDefaultOrder)
			Face = { vLess, vMore, vLess + 1 };
		else
			Face = { vLess, vLess + 1, vMore };
		ConnectionFaceSet.push_back(Face);
		vLess++;
	};

	int CheckPoint = 1;
	float AccumContain = ContainRate;
	while (LessCursor + 1 < LessEnd && MoreCursor + 1 < MoreEnd)
	{
		while (CheckPoint < AccumContain && MoreCursor + 1 < MoreEnd)
		{
			genFixLessFace(LessCursor, MoreCursor);
			CheckPoint++;
		}
		AccumContain += ContainRate;

		genFixMoreFace(LessCursor, MoreCursor);
	}

	while (LessCursor + 1 < LessEnd)
		genFixMoreFace(LessCursor, MoreCursor);
	while (MoreCursor + 1 < MoreEnd)
		genFixLessFace(LessCursor, MoreCursor);

	return ConnectionFaceSet;
}
