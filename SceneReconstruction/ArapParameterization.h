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
			int _VertexId;
			int _Prev;
			int _Next;
			int _Conj = -1;
		};

		struct SVertexInfo
		{
			std::vector<int> _InEdgeSet;
			bool _IsBoundary;
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
			Eigen::SparseMatrix<double, Eigen::ColMajor> __buildTutteSolveMatrix(const std::vector<SVertexInfo>& vVertexInfoSet);
			void __fillTutteSolveVectors(Eigen::VectorXd& vVectorX, Eigen::VectorXd& vVectorY, const CMesh& vMesh, const std::vector<bool>& vBoundaryStatus);
			Eigen::VectorXd __solveSparseMatrix(const Eigen::SparseMatrix<double, Eigen::ColMajor>& vMatrix, const Eigen::VectorXd& vVector);
			Eigen::MatrixXd __switch2UVMatrix(const CMesh& vMesh, const Eigen::VectorXd& vX, const Eigen::VectorXd& vY);

			Eigen::MatrixXd __solveARAP(const Eigen::MatrixXd& vVertexPos, const Eigen::MatrixXi& vFaces, const Eigen::MatrixXd& vInitialUV);
			int __findTwinRef(int vStartIndex, int vEndIndex);

			std::vector<SHalfEdge> m_HalfEdgeTable;
			std::vector<SVertexInfo> m_VertexInfoTable;
		};
	}
}

