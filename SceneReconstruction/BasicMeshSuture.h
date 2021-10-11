#pragma once
#include "MeshSuture.h"

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		class CBasicMeshSuture : public IMeshSuture
		{
		public:
			CBasicMeshSuture() = default;
			~CBasicMeshSuture() = default;

			virtual void sutureMeshes() override;
			void setCloud4SegmentPlane(PointCloud_t::Ptr vLHSCloud, PointCloud_t::Ptr vRHSCloud);
			void dumpMeshes(CMesh& voLHSMesh, CMesh& voRHSMesh);
		
		private:	
			void __executeIntersection(CMesh& vioMesh, const Eigen::Vector4f& vPlane, std::vector<int>& voDissociatedIndices, std::vector<SVertex>& voIntersectionPoints);
			std::vector<SVertex> __generatePublicVertices(std::vector<SVertex>& vLHSIntersectionPoints, std::vector<SVertex>& vRHSIntersectionPoints);
			void __connectVerticesWithMesh(CMesh& vioMesh, std::vector<int>& vDissociatedIndices, std::vector<SVertex>& vPublicVertices);
			void __removeUnreferencedVertex(CMesh& vioMesh);

			std::vector<SFace> __genConnectionFace(IndexType vNumLeft, IndexType vNumRight, bool vLeftBeforeRight, bool vIsClockwise = true);
			
			Eigen::Vector4f m_SegmentPlane = {};
		};
	}
}

