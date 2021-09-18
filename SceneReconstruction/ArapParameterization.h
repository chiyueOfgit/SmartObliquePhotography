#pragma once
#include "MeshParameterization.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
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
			CArapParameterization() = default;
			~CArapParameterization() = default;

			Eigen::MatrixXd execute();

			void buildHalfEdge();
			std::vector<bool> findBoundaryPoint();	//需要每顶点是否是边界，直接访问
			
			Eigen::MatrixXd calcInitialUV(const CMesh& vMesh, const std::vector<bool>& vBoundaryStatus);

		private:
			Eigen::SparseMatrix<double, Eigen::ColMajor, int> __buildTutteSolveMatrix(const std::vector<SHalfEdge>& vHalfEdgeSet);
			void __fillTutteSolveVectors(Eigen::VectorXd& vVectorX, Eigen::VectorXd& vVectorY, const CMesh& vMesh, const std::vector<bool>& vBoundaryStatus);
			Eigen::VectorXd __solveSparseMatrix(const Eigen::SparseMatrix<double, Eigen::ColMajor, int>& vMatrix, const Eigen::VectorXd& vVector);
			Eigen::MatrixXd __switch2UVMatrix(const CMesh& vMesh, const Eigen::VectorXd& vX, const Eigen::VectorXd& vY);

			Eigen::MatrixXd __solveARAP(const Eigen::MatrixXd& vVertexPos, const Eigen::MatrixXi& vFaces, Eigen::MatrixXd& vInitialUV);
			int __findTwinRef(int vStartIndex, int vEndIndex);

			std::vector<std::vector<int>> m_VertexInfoTable;
			std::vector<SHalfEdge> m_HalfEdgeTable;
		};
	}
}

