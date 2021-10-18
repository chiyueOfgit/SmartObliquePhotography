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

			void dumpIntersectionPoints(std::vector<SVertex>& vioIntersectionPoints) const;
			void dumpDissociatedPoints(std::vector<int>& vioDissociatedPoints) const;

		private:
			std::vector<int> m_DissociatedPoints;
			std::vector<SVertex> m_IntersectionPoints;
			Eigen::Vector3f m_Direction;
			
			std::vector<SVertex> __calcIntersectionPoints(const std::array<Eigen::Vector3f, 3>& vFace, const Eigen::Vector4f& vPlane) const;
			std::vector<int> __tellDissociatedPoint(const std::array<Eigen::Vector3f, 3>& vFace, const Eigen::Vector4f& vPlane) const;
			float __calcSignedDistance(const Eigen::Vector3f& vPoint, const Eigen::Vector4f& vPlane) const;
		};
	}
}