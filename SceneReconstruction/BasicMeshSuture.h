#pragma once
#include "MeshSuture.h"
#include <fstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>

namespace hiveObliquePhotography::SceneReconstruction
{
	class CBasicMeshSuture : public IMeshSuture
	{
	public:
		CBasicMeshSuture() = default;
		~CBasicMeshSuture() override = default;

		void sutureMeshesV() override;
		void dumpMeshes(CMesh& voLhsMesh, CMesh& voRhsMesh) const;

	private:
		std::vector<SVertex> __generatePublicVertices(const std::vector<SVertex>& vLhs, const std::vector<SVertex>& vRhs);
		void __connectVerticesWithMesh(const std::vector<int>& vDissociatedIndices, const std::vector<SVertex>& vPublicVertices, CMesh& vioMesh);
		void __removeUnreferencedVertex(CMesh& vioMesh);
		SVertex __findNearestPoint(const std::vector<SVertex>& vVectexSet, const SVertex& vOrigin);
		std::vector<SFace> __genConnectionFace(const CMesh& vMesh, const std::vector<int>& vLeft, const std::vector<int>& vRight, bool vIsClockwise = true);

		void __sortDissociatedIndices(const CMesh& vMesh, std::vector<int>& vioDissociatedPoints);
		void __sortIntersectionPoints(std::vector<SVertex>& vioIntersectionPoints);
		void __sortByVertexLoop(std::vector<int>& vioOrderIndices, const std::vector<SVertex>& vVertexSet);
		Eigen::Vector3f __calcUpVector(const CMesh& vMesh);

		Eigen::Vector4f m_SegmentPlane = Eigen::Vector4f::Zero();
		Eigen::Vector3f m_Direction = Eigen::Vector3f::Zero();
		
#ifdef _DEBUG
		void __serializeIndices(const std::vector<int>& vData, const std::string& vFileName) const
		{
			std::ofstream Out(vFileName);
			boost::archive::text_oarchive Oarchive(Out);
			Oarchive& BOOST_SERIALIZATION_NVP(vData);
			Out.close();
		}
#endif
	};
}
