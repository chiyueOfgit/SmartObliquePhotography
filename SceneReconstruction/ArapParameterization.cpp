#include "pch.h"
#include "ArapParameterization.h"

using namespace hiveObliquePhotography::SceneReconstruction;

_REGISTER_NORMAL_PRODUCT(CArapParameterization, KEYWORD::ARAP_MESH_PARAMETERIZATION)

using namespace hiveObliquePhotography::SceneReconstruction;

CArapParameterization::CArapParameterization()
{
	m_VertexInfoTable.resize(m_Mesh.m_Vertices.size());
}

//*****************************************************************
//FUNCTION: 
Eigen::MatrixXd CArapParameterization::execute()
{
	auto BoundaryStatus = findBoundaryPoint();
	auto InitialUV = calcInitialUV(m_Mesh, BoundaryStatus);
	
	return InitialUV;
}

//*****************************************************************
//FUNCTION: 
void CArapParameterization::buildHalfEdge()
{
	
}

//*****************************************************************
//FUNCTION: 
std::vector<bool> CArapParameterization::findBoundaryPoint()
{
	return {};
}

//*****************************************************************
//FUNCTION: 
Eigen::MatrixXd CArapParameterization::calcInitialUV(const CMesh& vMesh, const std::vector<bool>& vBoundaryStatus)
{
	auto TutteMatrix = __calcTutteSolveMatrix(vMesh, vBoundaryStatus);
	Eigen::VectorXd VectorX, VectorY;
	__fillTutteSolveVectors(VectorX, VectorY, vMesh, vBoundaryStatus);
	auto X = __solveSparseMatrix(TutteMatrix, VectorX);
	auto Y = __solveSparseMatrix(TutteMatrix, VectorY);
	
	return __switch2UVMatrix(vMesh, X, Y);
}

//*****************************************************************
//FUNCTION: 
Eigen::SparseMatrix<double, Eigen::ColMajor, int> CArapParameterization::__calcTutteSolveMatrix(const CMesh& vMesh, const std::vector<bool>& vBoundaryStatus)
{
	auto NumVertices = vMesh.m_Vertices.size();
	Eigen::SparseMatrix<double, Eigen::ColMajor, int> TutteMatrix(NumVertices, NumVertices);
	for (int VertexId = 0; VertexId < NumVertices; VertexId++)
	{
		if (vBoundaryStatus[VertexId])	//是边界
			TutteMatrix.insert(VertexId, VertexId) = 1.0;
		else
		{
			auto FaceSet = vMesh.findFacesByVertex(VertexId);
			std::map<int, bool> Neighbors;
			for (auto Face : FaceSet)
				for (int i = 0; i < 3; i++)
					Neighbors[Face[i]] = true;
			auto NumNeighbors = Neighbors.size() - 1;	//减去自己
			
			TutteMatrix.insert(VertexId, VertexId) = -1.0 * NumNeighbors;
			
			for (auto Pair : Neighbors)
				if (Pair.first != VertexId && !vBoundaryStatus[Pair.first])	//不为自己且不为边界
					TutteMatrix.insert(VertexId, Pair.first) = 1.0;
		}
	}

	return TutteMatrix;
}

//*****************************************************************
//FUNCTION: 
void CArapParameterization::__fillTutteSolveVectors(Eigen::VectorXd& vVectorX, Eigen::VectorXd& vVectorY, const CMesh& vMesh, const std::vector<bool>& vBoundaryStatus)
{
	auto NumVertices = vMesh.m_Vertices.size();
	vVectorX.resize(NumVertices);
	vVectorY.resize(NumVertices);
	for (int VertexId = 0; VertexId < NumVertices; VertexId++)
	{
		if (vBoundaryStatus[VertexId])
		{
			vVectorX(VertexId) = vMesh.m_Vertices[VertexId].x;
			vVectorY(VertexId) = vMesh.m_Vertices[VertexId].y;
		}
		else
		{
			vVectorX(VertexId) = 0.0;
			vVectorY(VertexId) = 0.0;
		}	
	}
}

//*****************************************************************
//FUNCTION: 
Eigen::VectorXd CArapParameterization::__solveSparseMatrix(const Eigen::SparseMatrix<double, Eigen::ColMajor, int>& vMatrix, const Eigen::VectorXd& vVector)
{
	auto CompressMatrix = vMatrix;
	CompressMatrix.makeCompressed();

	Eigen::SimplicialLDLT<Eigen::SparseMatrix<double, Eigen::ColMajor>> Solver;
	Solver.analyzePattern(CompressMatrix);
	Solver.factorize(CompressMatrix);
	_ASSERTE(Solver.info() == Eigen::Success);
	auto Solution = Solver.solve(vVector);
	_ASSERTE(solverA.info() == Eigen::Success);
	return Solution;
}

//*****************************************************************
//FUNCTION: 
Eigen::MatrixXd CArapParameterization::__switch2UVMatrix(const CMesh& vMesh, const Eigen::VectorXd& vX, const Eigen::VectorXd& vY)
{
	_ASSERTE(vX.size() == vMesh.m_Vertices.size() && vX.size() == vY.size());
	Eigen::MatrixXd UVMatrix(vMesh.m_Vertices.size(), 2);
	auto BoundingBox = vMesh.calcAABB();
	float WidthU = BoundingBox.second.x() - BoundingBox.first.x();
	float HeightV = BoundingBox.second.y() - BoundingBox.first.y();
	float BeginX = BoundingBox.first.x(), BeginY = BoundingBox.first.y();

	for (int VertexId = 0; VertexId < vX.size(); VertexId++)
	{
		float U = (vX(VertexId) - BeginX) / WidthU;
		float V = (vY(VertexId) - BeginY) / HeightV;

		UVMatrix.row(VertexId) = Eigen::Vector2d(U, V);
	}

	return UVMatrix;
}

//*****************************************************************
//FUNCTION: 
Eigen::MatrixXd CArapParameterization::__solveARAP(const Eigen::MatrixXd& vVertexPos, const Eigen::MatrixXi& vFaces, Eigen::MatrixXd& vInitialUV)
{
	
}