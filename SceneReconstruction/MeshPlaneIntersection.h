#pragma once
#include "Mesh.h"

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		class CMeshPlaneIntersection
		{
		public:
			CMeshPlaneIntersection() = default;
			~CMeshPlaneIntersection() = default;

			void execute(CMesh& vioMesh, const Eigen::Vector4f& vPlane);

			void dumpIntersectionPoints(std::vector<SVertex>& vioIntersectionPoints);
			void dumpDissociatedMesh(CMesh& vioDissociatedMesh);

		private:
			CMesh m_DissociatedMesh;
			std::vector<SVertex> m_IntersectionPoints;
		};
	}
}