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
			void dumpDissociatedPoints(std::vector<SVertex>& vioDissociatedPoints);

		private:
			std::vector<SVertex> m_DissociatedPoints;
			std::vector<SVertex> m_IntersectionPoints;
		};
	}
}