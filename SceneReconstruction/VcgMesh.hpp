#pragma once
#include <vcg/complex/complex.h>
#include "Mesh.h"

namespace hiveObliquePhotography
{
	struct SVcgVertex; struct SVcgFace; struct SVcgEdge;
	struct SVcgUsedTypes : public vcg::UsedTypes<	vcg::Use<SVcgVertex>::AsVertexType,
		vcg::Use<SVcgFace>::AsFaceType,
		vcg::Use<SVcgEdge>::AsEdgeType>
	{};
	struct SVcgVertex : public vcg::Vertex<SVcgUsedTypes, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::TexCoord2f, vcg::vertex::BitFlags, vcg::vertex::Mark> {};
	struct SVcgFace : public vcg::Face<SVcgUsedTypes, vcg::face::FFAdj, vcg::face::VertexRef, vcg::face::BitFlags, vcg::face::Mark> {};
	struct SVcgEdge : public vcg::Edge<SVcgUsedTypes> {};
	class CVcgMesh : public vcg::tri::TriMesh<std::vector<SVcgVertex>, std::vector<SVcgFace>, std::vector<SVcgEdge>> {};

	inline void fromVcgMesh(const CVcgMesh& vFrom, CMesh& vTo)
	{
		vTo.m_Vertices.clear();
		vTo.m_Vertices.reserve(vFrom.VN());
		vTo.m_Faces.clear();
		vTo.m_Faces.reserve(vFrom.FN());
		
		for (auto& Vertex : vFrom.vert)
			vTo.m_Vertices.emplace_back(
				Vertex.P().X(), Vertex.P().Y(), Vertex.P().Z(),
				Vertex.N().X(), Vertex.N().Y(), Vertex.N().Z(),
				Vertex.T().U(), Vertex.T().V()
			);

		auto Begin = vFrom.vert.data();
		for (auto& Face : vFrom.face)
			vTo.m_Faces.emplace_back(Face.V(0) - Begin, Face.V(1) - Begin, Face.V(2) - Begin);
	}

	inline void toVcgMesh(const CMesh& vFrom, CVcgMesh& vTo)
	{
		vTo.Clear();

		auto VertexIterator = vcg::tri::Allocator<CVcgMesh>::AddVertices(vTo, vFrom.m_Vertices.size());
		for (auto& Vertex : vFrom.m_Vertices)
		{
			VertexIterator->P() = { Vertex.x, Vertex.y, Vertex.z };
			VertexIterator->N() = { Vertex.nx, Vertex.ny, Vertex.nz };
			VertexIterator->T() = { Vertex.u, Vertex.v };
			++VertexIterator;
		}

		auto FaceIterator = vcg::tri::Allocator<CVcgMesh>::AddFaces(vTo, vFrom.m_Faces.size());
		for (auto& Face : vFrom.m_Faces)
		{
			FaceIterator->V(0) = &vTo.vert.at(Face.a);
			FaceIterator->V(1) = &vTo.vert.at(Face.b);
			FaceIterator->V(2) = &vTo.vert.at(Face.c);
			++FaceIterator;
		}
		
		vcg::tri::UpdateTopology<CVcgMesh>::FaceFace(vTo);
	}
}
