#include "pch.h"
#include "ArapParameterizer.h"

#include <igl/arap.h>
#include <igl/readOBJ.h>
#include <igl/boundary_loop.h>

#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/vector.hpp>
#include <iomanip>
#include <Eigen/IterativeLinearSolvers>


using namespace hiveObliquePhotography::SceneReconstruction;

_REGISTER_NORMAL_PRODUCT(CArapParameterizer, KEYWORD::ARAP_MESH_PARAMETERIZATION)

using namespace hiveObliquePhotography::SceneReconstruction;

//*****************************************************************
//FUNCTION: 
Eigen::MatrixXd CArapParameterizer::execute()
{
	buildHalfEdge();
	auto BoundarySet = findBoundaryPoint();
	std::vector<bool> BoundaryStatus(m_Mesh.m_Vertices.size(), false);
	for (auto& Index : BoundarySet)
		BoundaryStatus[Index] = true;
	
	auto InitialUV = calcInitialUV(m_Mesh, BoundaryStatus);
	
	////NOTE:利用ARAP方法得到UV；
	//auto UV = __solveARAP(m_Mesh.getVerticesMatrix(), m_Mesh.getFacesMatrix(), InitialUV, BoundarySet);

	return InitialUV;
}

//*****************************************************************
//FUNCTION: 
void CArapParameterizer::buildHalfEdge()
{
	m_VertexInfoTable.resize(m_Mesh.m_Vertices.size());
	m_HalfEdgeTable.clear();
	m_HalfEdgeTable.reserve(m_Mesh.m_Faces.size() * 3);
	std::vector Traversed(m_Mesh.m_Vertices.size(), false);
	for(size_t FaceId = 0; FaceId < m_Mesh.m_Faces.size(); ++FaceId)
	{
		auto& Face = m_Mesh.m_Faces[FaceId];
		const auto& VertexA = m_Mesh.m_Vertices[Face[0]];
		const auto& VertexB = m_Mesh.m_Vertices[Face[1]];
		const auto& VertexC = m_Mesh.m_Vertices[Face[2]];
		Eigen::Vector3f FaceNormal = (VertexC.xyz() - VertexB.xyz()).cross(VertexA.xyz() - VertexB.xyz());

		if (VertexA.normal().dot(FaceNormal) < 0 && VertexB.normal().dot(FaceNormal) < 0 && VertexC.normal().dot(FaceNormal) < 0)
			std::swap(Face[1], Face[2]);

		for(size_t i = 0; i < 3; ++i)
		{
			SHalfEdge HalfEdge;
			HalfEdge._VertexId = Face[i];
			HalfEdge._Face = FaceId;
			auto Index = m_HalfEdgeTable.size();
			m_VertexInfoTable[Face[i]].push_back(Index);
			HalfEdge._Prev = Index + ((i == 0) ? (2) : (-1));
			HalfEdge._Next = Index + ((i == 2) ? (-2) : (1));
			if(Traversed[Face[i]] && Traversed[Face[(i + 1) % 3]])  
			{
				HalfEdge._Conj = __findTwinRef(Face[i], Face[(i + 1) % 3]);
				if(HalfEdge._Conj >= 0)
					m_HalfEdgeTable[HalfEdge._Conj]._Conj = Index;
			}
			m_HalfEdgeTable.push_back(HalfEdge);
		}

		Traversed[Face[0]] = true;
		Traversed[Face[1]] = true;
		Traversed[Face[2]] = true;
	}
}

//*****************************************************************
//FUNCTION: 寻找边界点；
std::vector<int> CArapParameterizer::findBoundaryPoint()
{
	Eigen::MatrixXi F = m_Mesh.getFacesMatrix();
	std::vector<int> Boundary;
	igl::boundary_loop(F, Boundary);

	return Boundary;
}

//*****************************************************************
//FUNCTION: 
Eigen::MatrixXd CArapParameterizer::calcInitialUV(const CMesh& vMesh, const std::vector<bool>& vBoundaryStatus)
{
	auto TutteMatrix = __buildTutteSolveMatrix(m_HalfEdgeTable, vBoundaryStatus);
	Eigen::VectorXd VectorX, VectorY;
	__fillTutteSolveVectors(VectorX, VectorY, vMesh, vBoundaryStatus);
	auto X = __solveSparseMatrix(TutteMatrix, VectorX);
	auto Y = __solveSparseMatrix(TutteMatrix, VectorY);
	
	return __switch2UVMatrix(vMesh, X, Y);
}

//*****************************************************************
//FUNCTION: 
Eigen::SparseMatrix<double, Eigen::ColMajor> CArapParameterizer::__buildTutteSolveMatrix(const std::vector<SHalfEdge>& vHalfEdgeSet, const std::vector<bool>& vBoundaryStatus)
{
	auto NumVertices = m_Mesh.m_Vertices.size();
	Eigen::SparseMatrix<double, Eigen::ColMajor> TutteMatrix(NumVertices, NumVertices);

	typedef Eigen::Triplet<double> TWeight;
	std::vector<TWeight> WeightTriplet;

	auto MaxNonZeroValueAmountOfRow = [&]()
	{
		int MaxNumber = 0;
		for (size_t VertexId = 0; VertexId < NumVertices; ++VertexId)
		{
			if (!vBoundaryStatus[VertexId]) //interior
			{
				if (MaxNumber < m_VertexInfoTable[VertexId].size())
					MaxNumber = m_VertexInfoTable[VertexId].size();
			}
		}

		return MaxNumber;
	};
	
	WeightTriplet.reserve(NumVertices * MaxNonZeroValueAmountOfRow());

	//Note:均匀权重方案；
	auto Uniform = []()
	{
		return 1.0;
	};

	//Note：平均权重方案；
	auto MeanWalue = [&](int vHalfEdge, int vVertex, int vNextVertex)
	{
		auto CalcAngle = [&](int vFaceId) -> double
		{
			auto Face = m_Mesh.m_Faces[vFaceId];
			int RestVertex = 0;
			for (int i = 0; i < 3; i++)
			{
				auto VertexId = Face[i];
				if (VertexId != vVertex && VertexId != vNextVertex)
				{
					RestVertex = VertexId;
					break;
				}
			}

			auto A = m_Mesh.m_Vertices[vNextVertex].xyz() - m_Mesh.m_Vertices[vVertex].xyz();
			auto B = m_Mesh.m_Vertices[RestVertex].xyz() - m_Mesh.m_Vertices[vVertex].xyz();

			return std::acos(A.dot(B) / (A.norm() * B.norm()));
		};

		auto Sigma = CalcAngle(vHalfEdgeSet[vHalfEdge]._Face);
		auto Gamma = CalcAngle(vHalfEdgeSet[vHalfEdgeSet[vHalfEdge]._Conj]._Face);
		auto Length = (m_Mesh.m_Vertices[vVertex].xyz() - m_Mesh.m_Vertices[vNextVertex].xyz()).norm();

		return (std::tan(Sigma / 2) + std::tan(Gamma / 2)) / Length;
	};

	for (size_t VertexId = 0; VertexId < NumVertices; ++VertexId)
	{
		if (vBoundaryStatus[VertexId])
			WeightTriplet.push_back(TWeight(VertexId, VertexId, 1.0));
		else 
		{
			const auto& NeighborHalfEdgeSet = m_VertexInfoTable[VertexId];

			double SumWeight = 0;
			for (auto i : NeighborHalfEdgeSet)
			{
				auto NextVertexId = vHalfEdgeSet[vHalfEdgeSet[i]._Next]._VertexId;
				auto Weight = Uniform();
				//auto Weight = MeanWalue(i, VertexId, NextVertexId);
				WeightTriplet.push_back(TWeight(VertexId, NextVertexId, Weight));
				SumWeight += Weight;
			}

			WeightTriplet.push_back(TWeight(VertexId, VertexId, -1.0 * SumWeight));
		}
	}

	TutteMatrix.setFromTriplets(WeightTriplet.begin(), WeightTriplet.end());
	return TutteMatrix;
}

//*****************************************************************
//FUNCTION: 
void CArapParameterizer::__fillTutteSolveVectors(Eigen::VectorXd& vVectorX, Eigen::VectorXd& vVectorY, const CMesh& vMesh, const std::vector<bool>& vBoundaryStatus)
{
	auto NumVertices = vMesh.m_Vertices.size();
	vVectorX.resize(NumVertices);
	vVectorY.resize(NumVertices);
	std::pair<int, int> XYAxis;
	int HeightAxis;
	vMesh.calcModelPlaneAxis(XYAxis, HeightAxis);
	for (int VertexId = 0; VertexId < NumVertices; VertexId++)
	{
		if (vBoundaryStatus[VertexId])
		{
			vVectorX(VertexId) = vMesh.m_Vertices[VertexId][XYAxis.first];
			vVectorY(VertexId) = vMesh.m_Vertices[VertexId][XYAxis.second];
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
Eigen::VectorXd CArapParameterizer::__solveSparseMatrix(const Eigen::SparseMatrix<double, Eigen::ColMajor>& vMatrix, const Eigen::VectorXd& vVector)
{
	auto CompressMatrix = vMatrix;
	CompressMatrix.makeCompressed();
	
	Eigen::BiCGSTAB<Eigen::SparseMatrix<double, Eigen::ColMajor>>Solver;
	Solver.analyzePattern(CompressMatrix);
	Solver.factorize(CompressMatrix);
	auto Solution = Solver.solve(vVector);

	switch (Solver.info())
	{
	case Eigen::Success:
		hiveEventLogger::hiveOutputEvent("Computation was successful.");
		break;
	case Eigen::NumericalIssue:
		hiveEventLogger::hiveOutputEvent("The provided data did not satisfy the prerequisites.");
		break;
	case Eigen::NoConvergence:
		hiveEventLogger::hiveOutputEvent("Iterative procedure did not converge.");
		break;
	case Eigen::InvalidInput:
		hiveEventLogger::hiveOutputEvent("The inputs are invalid, or the algorithm has been improperly called.");
		break;
	default:
		hiveEventLogger::hiveOutputEvent("Unexpected error.");
		break;
	}
	
	return Solution;
}

//*****************************************************************
//FUNCTION: 
Eigen::MatrixXd CArapParameterizer::__switch2UVMatrix(const CMesh& vMesh, const Eigen::VectorXd& vX, const Eigen::VectorXd& vY)
{
	_ASSERTE(vX.size() == vMesh.m_Vertices.size() && vX.size() == vY.size());
	Eigen::MatrixXd UVMatrix(vMesh.m_Vertices.size(), 2);
	auto BoundingBox = vMesh.calcAABB();
	std::pair<int, int> XYAxis;
	int HeightAxis;
	vMesh.calcModelPlaneAxis(XYAxis, HeightAxis);
	float WidthU = BoundingBox.second.data()[XYAxis.first] - BoundingBox.first.data()[XYAxis.first];
	float HeightV = BoundingBox.second.data()[XYAxis.second] - BoundingBox.first.data()[XYAxis.second];
	float BeginX = BoundingBox.first.data()[XYAxis.first], BeginY = BoundingBox.first.data()[XYAxis.second];

	for (int VertexId = 0; VertexId < vX.size(); VertexId++)
	{
		float U = (vX(VertexId) - BeginX) / WidthU;
		float V = (vY(VertexId) - BeginY) / HeightV;

		UVMatrix.row(VertexId) = Eigen::Vector2d(U, V);
	}

	return UVMatrix;
}

//*****************************************************************
//FUNCTION: 寻找相反的半边；
int CArapParameterizer::__findTwinRef(int vStartIndex, int vEndIndex)
{
	for(auto EdgeIndex : m_VertexInfoTable[vEndIndex])
		if (m_HalfEdgeTable[m_HalfEdgeTable[EdgeIndex]._Next]._VertexId == vStartIndex)
			return EdgeIndex;
	return -1;
}
