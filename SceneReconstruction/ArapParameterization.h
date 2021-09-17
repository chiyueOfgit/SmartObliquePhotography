#pragma once
#include "MeshParameterization.h"
#include <Eigen/src/SparseCore/SparseMatrix.h>

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		struct SVertexInfo
		{
			int VertexRef;
			int HalfEdgeRef;
		};

		struct SHalfEdge
		{
			int VertexRef;
			int Prev;
			int Next;
			int Twin = -1;
		};

		class CArapParameterization : public IMeshParameterization
		{
		public:
			CArapParameterization();
			~CArapParameterization() = default;

			void buildHalfEdge();
			std::vector<int> findBoundaryPoint();
			
			Eigen::MatrixXd calcInitialUV(const CMesh& vMesh);

		private:
			void __fillTutteSolveMatrix(Eigen::SparseMatrix<double, Eigen::ColMajor, int>& vMatrix, const CMesh& vMesh);
			void __fillTutteSolveVectors(Eigen::VectorXd& vVectorU, Eigen::VectorXd& vVectorV, const CMesh& vMesh);
			Eigen::VectorXd __solveTutteEmbedding(const Eigen::SparseMatrix<double, Eigen::ColMajor, int>& vMatrix, const Eigen::VectorXd& vVector);

			Eigen::MatrixXd __solveARAP(const Eigen::MatrixXd& vVertexPos, const Eigen::MatrixXi& vFaces, Eigen::MatrixXd& vInitialUV);

			std::vector<SVertexInfo> m_VertexInfoTable;
			std::vector<SHalfEdge> m_HalfEdgeTable;
		};
	}
}

