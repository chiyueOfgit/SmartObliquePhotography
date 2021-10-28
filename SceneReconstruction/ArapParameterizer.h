#pragma once
#include "MeshParameterizer.h"
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

			bool execute(Eigen::MatrixXd& voUV) override;

			void buildHalfEdge();
			std::vector<int> findBoundaryPoint();
			
			bool calcInitialUV(const CMesh& vMesh, const std::vector<bool>& vBoundaryStatus, Eigen::MatrixXd& voUV);
		private:
			Eigen::SparseMatrix<double, Eigen::ColMajor> __buildTutteSolveMatrix(const std::vector<SHalfEdge>& vHalfEdgeSet, const std::vector<bool>& vBoundaryStatus);
			void __fillTutteSolveVectors(Eigen::VectorXd& vVectorX, Eigen::VectorXd& vVectorY, const CMesh& vMesh, const std::vector<bool>& vBoundaryStatus);
			bool __solveSparseMatrix(const Eigen::SparseMatrix<double, Eigen::ColMajor>& vMatrix, const Eigen::VectorXd& vVector, Eigen::VectorXd& voAnswer);
			Eigen::MatrixXd __switch2UVMatrix(const CMesh& vMesh, const Eigen::VectorXd& vX, const Eigen::VectorXd& vY);
			int __findTwinRef(int vStartIndex, int vEndIndex);
			
			std::vector<SHalfEdge> m_HalfEdgeTable;
			std::vector<std::vector<int>> m_VertexInfoTable;
		};
	}
}

