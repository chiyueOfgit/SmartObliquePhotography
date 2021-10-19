#pragma once
#include "MeshSuture.h"

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
		void __executeIntersection(CMesh& vioMesh, const Eigen::Vector4f& vPlane, std::vector<SVertex>& voIntersectionPoints, std::vector<int>& voDissociatedIndices);
		void __generatePublicVertices(const std::vector<SVertex>& vLhs, const std::vector<SVertex>& vRhs, std::vector<SVertex>& voPublicVertices);
		void __connectVerticesWithMesh(CMesh& vioMesh, std::vector<int>& vDissociatedIndices, std::vector<SVertex>& vPublicVertices, const Eigen::Vector3f& vDirection);
		void __removeUnreferencedVertex(CMesh& vioMesh);
		SVertex __findNearestPoint(const std::vector<SVertex>& vVectexSet, const SVertex& vOrigin);
		std::vector<SFace> __genConnectionFace(const CMesh& vMesh, const std::vector<int>& vLeft, const std::vector<int>& vRight, const Eigen::Vector3f& vDirection, bool vIsClockwise = true);
		void __serializeIndices(const std::vector<int>& vData, const std::string& vFileName) const;

		void __sortDissociatedIndices(const CMesh& vMesh, std::vector<int>& vioDissociatedPoints, Eigen::Vector3f& vDirection);
		void __sortIntersectionPoints(std::vector<SVertex>& vioIntersectionPoints, Eigen::Vector3f& vDirection);
		void __sortByVertexLoop(std::vector<int>& vioOrderIndices, const std::vector<SVertex>& vVertexSet);
		void __findSutureDirection(const CMesh& vMesh, Eigen::Vector3f& voDirection);
		
		Eigen::Vector4f m_SegmentPlane = {};
	};
}
