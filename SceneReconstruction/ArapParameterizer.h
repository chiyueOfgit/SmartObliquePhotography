#pragma once
#include "MeshParameterizer.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <set>

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		struct SHalfEdge
		{
			int _VertexId;
			int _Prev;
			int _Next;
			int _Conj = -1;
			int _Face;
		};

		class CArapParameterizer : public IMeshParameterizer
		{
		public:
			CArapParameterizer() = default;
			~CArapParameterizer() = default;

			Eigen::MatrixXd execute();

			void buildHalfEdge();
			std::vector<int> findBoundaryPoint();
			
			Eigen::MatrixXd calcInitialUV(const CMesh& vMesh, const std::vector<bool>& vBoundaryStatus);
			void setPath4Boundary(const std::string& vPath) { m_MeshPath = vPath; }
		private:
			Eigen::SparseMatrix<double, Eigen::ColMajor> __buildTutteSolveMatrix(const std::vector<SHalfEdge>& vHalfEdgeSet, const std::vector<bool>& vBoundaryStatus);
			void __fillTutteSolveVectors(Eigen::VectorXd& vVectorX, Eigen::VectorXd& vVectorY, const CMesh& vMesh, const std::vector<bool>& vBoundaryStatus);
			Eigen::VectorXd __solveSparseMatrix(const Eigen::SparseMatrix<double, Eigen::ColMajor>& vMatrix, const Eigen::VectorXd& vVector);
			Eigen::MatrixXd __switch2UVMatrix(const CMesh& vMesh, const Eigen::VectorXd& vX, const Eigen::VectorXd& vY);

			Eigen::MatrixXd __solveARAP(const Eigen::MatrixXd& vVertexPos, const Eigen::MatrixXi& vFaces, const Eigen::MatrixXd& vInitialUV, const std::vector<int>& vBoundarySet);
			int __findTwinRef(int vStartIndex, int vEndIndex);
			void __findValidBoundary(std::set<int>& vBoundarySet, std::vector<int>& voValidBoundary);
			void __normalizeUV(Eigen::MatrixXd& vioUVMatrix);
			
			std::vector<SHalfEdge> m_HalfEdgeTable;
			std::vector<std::vector<int>> m_VertexInfoTable;
			std::string m_MeshPath;
		};
	}
}

