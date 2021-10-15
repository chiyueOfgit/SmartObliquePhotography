#pragma once
#include "MeshSuture.h"
#include "MeshPlaneIntersection.h"

namespace hiveObliquePhotography::SceneReconstruction
{
	class CBasicMeshSuture : public IMeshSuture
	{
	public:
		CBasicMeshSuture() = default;
		~CBasicMeshSuture() override = default;

		void sutureMeshesV() override;
		void setCloud4SegmentPlane(PointCloud_t::ConstPtr vLhs, PointCloud_t::ConstPtr vRhs);
		void dumpMeshes(CMesh& voLhsMesh, CMesh& voRhsMesh) const;

	private:
		void __executeIntersection(CMesh& vioMesh, const Eigen::Vector4f& vPlane, std::vector<int>& voDissociatedIndices, std::vector<SVertex>& voIntersectionPoints);
		void __generatePublicVertices(const std::vector<SVertex>& vLhs, const std::vector<SVertex>& vRhs, std::vector<SVertex>& voPublicVertices);
		void __connectVerticesWithMesh(CMesh& vioMesh, std::vector<int>& vDissociatedIndices, std::vector<SVertex>& vPublicVertices);
		void __removeUnreferencedVertex(CMesh& vioMesh);
		SVertex __findNearestPoint(const std::vector<SVertex>& vVectexSet, const SVertex& vOrigin);
		std::vector<SFace> __genConnectionFace(IndexType vNumLeft, IndexType vNumRight, bool vLeftBeforeRight, bool vIsClockwise = true);
		void __serializeIndices(const std::vector<int>& vData, const std::string& vFileName) const;

		CMeshPlaneIntersection m_MeshPlaneIntersection;
		Eigen::Vector4f m_SegmentPlane = {};
	};
}
