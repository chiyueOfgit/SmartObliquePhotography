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
			void dumpMeshes(CMesh& voLHSMesh, CMesh& voRHSMesh);
		
		private:	
			Eigen::Vector4f __calcSegmentPlane();
			void __executeIntersection(CMesh& vioMesh, const Eigen::Vector4f& vPlane, std::vector<int>& voDissociatedIndices, std::vector<SVertex>& voIntersectionPoints);
			std::vector<SVertex> __generatePublicVertices(std::vector<SVertex>& vLHSIntersectionPoints, std::vector<SVertex>& vRHSIntersectionPoints);
			void __connectVerticesWithMesh(CMesh& vioMesh, std::vector<int>& vDissociatedIndices, std::vector<SVertex>& vPublicVertices);
			void __removeUnreferencedVertex(CMesh& vioMesh);
		};
	}
}

