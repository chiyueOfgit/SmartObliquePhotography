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
			void dumpDissociatedPoints(std::vector<int>& vioDissociatedPoints);

		private:
			std::vector<int> m_DissociatedPoints;
			std::vector<SVertex> m_IntersectionPoints;

			std::vector<SVertex> __calcIntersectionPoints(const std::vector<Eigen::Vector3f>& vFace, const Eigen::Vector4f& vPlane);
			std::vector<int> __tellDissociatedPoint(const std::vector<Eigen::Vector3f>& vFace, const Eigen::Vector4f& vPlane);
			Eigen::Vector3f __generateDefaultPlanePoint(const Eigen::Vector4f& vPlane);
			
			void __sortDissociatedIndices(const CMesh& vMesh, std::vector<int>& vioDissociatedPoints, Eigen::Vector3f& vDirection);
			void __sortIntersectionPoints(std::vector<SVertex>& vioIntersectionPoints, Eigen::Vector3f& vDirection);
			void __findHightAxis(const CMesh& vMesh, Eigen::Vector3f& voHightAxis);
			
		};
	}
}