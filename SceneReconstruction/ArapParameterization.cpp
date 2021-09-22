#include "pch.h"
#include "ArapParameterization.h"
#include <set>
#include <igl/arap.h>

#include <fstream>	//remove
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/set.hpp>

using namespace hiveObliquePhotography::SceneReconstruction;

_REGISTER_NORMAL_PRODUCT(CArapParameterization, KEYWORD::ARAP_MESH_PARAMETERIZATION)

using namespace hiveObliquePhotography::SceneReconstruction;

//*****************************************************************
//FUNCTION: 
Eigen::MatrixXd CArapParameterization::execute()
{
	buildHalfEdge();
	auto BoundaryStatus = findBoundaryPoint();
	auto InitialUV = calcInitialUV(m_Mesh, BoundaryStatus);
	//auto UV = __solveARAP(m_Mesh.getVerticesMatrix(), m_Mesh.getFacesMatrix(), InitialUV);
	return InitialUV;
}

//*****************************************************************
//FUNCTION: 
void CArapParameterization::buildHalfEdge()
{
	m_VertexInfoTable.resize(m_Mesh.m_Vertices.size());
	m_HalfEdgeTable.clear();
	m_HalfEdgeTable.reserve(m_Mesh.m_Vertices.size() * 3);
	std::vector<bool> Flag(m_Mesh.m_Vertices.size(), false);
	for(auto& Face : m_Mesh.m_Faces)
	{
		for(int i = 0; i < 3; i++)
		{
			SHalfEdge HalfEdge;
			HalfEdge._VertexId = Face[i];
			auto Index = m_HalfEdgeTable.size();
			m_VertexInfoTable[Face[i]].push_back(Index);
			HalfEdge._Prev = Index + ((i == 0) ? (2) : (-1));
			HalfEdge._Next = Index + ((i == 2) ? (-2) : (1));
			if(Flag[Face[i]] && Flag[Face[(i + 1) % 3]])
			{
				HalfEdge._Conj = __findTwinRef(Face[i], Face[(i + 1) % 3]);
				if( HalfEdge._Conj >= 0 )
				    m_HalfEdgeTable[HalfEdge._Conj]._Conj = Index;
			}
			m_HalfEdgeTable.push_back(HalfEdge);
		}
		Flag[Face[0]] = true;
		Flag[Face[1]] = true;
		Flag[Face[2]] = true;
	}
}

//*****************************************************************
//FUNCTION: 
std::vector<bool> CArapParameterization::findBoundaryPoint()
{
	std::vector<bool> OutPutSet(m_Mesh.m_Vertices.size(), false);
	std::set<int> BoundarySet;
	for(auto& HalfEdge : m_HalfEdgeTable)
	{
		if(HalfEdge._Conj < 0)
		{
			OutPutSet[HalfEdge._VertexId] = true;
			OutPutSet[m_HalfEdgeTable[HalfEdge._Next]._VertexId] = true;

			BoundarySet.insert(HalfEdge._VertexId);
			BoundarySet.insert(m_HalfEdgeTable[HalfEdge._Next]._VertexId);
		}
	}

	std::ofstream file("BoundaryPoints.txt");
	boost::archive::text_oarchive oa(file);
	oa& BOOST_SERIALIZATION_NVP(BoundarySet);
	file.close();

	return OutPutSet;
}

//*****************************************************************
//FUNCTION: 
Eigen::MatrixXd CArapParameterization::calcInitialUV(const CMesh& vMesh, const std::vector<bool>& vBoundaryStatus)
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
Eigen::SparseMatrix<double, Eigen::ColMajor> CArapParameterization::__buildTutteSolveMatrix(const std::vector<SHalfEdge>& vHalfEdgeSet, const std::vector<bool>& vBoundaryStatus)
{
	auto NumVertices = m_Mesh.m_Vertices.size();
	Eigen::SparseMatrix<double, Eigen::ColMajor> TutteMatrix(NumVertices, NumVertices);
	TutteMatrix.reserve(Eigen::VectorXd::Zero(NumVertices));
	for (size_t VertexId = 0; VertexId < NumVertices; ++VertexId)
	{
		if (vBoundaryStatus[VertexId]) //boundary
			TutteMatrix.insert(VertexId, VertexId) = 1.0;
		else //interior
		{
			const auto& NeighborHalfEdgeSet = m_VertexInfoTable[VertexId];

			TutteMatrix.insert(VertexId, VertexId) = -1.0 * NeighborHalfEdgeSet.size();

			std::set<int> NeighborVertexSet;
			for (auto i : NeighborHalfEdgeSet)
				NeighborVertexSet.insert(vHalfEdgeSet[vHalfEdgeSet[i]._Next]._VertexId);

			for (auto NextVertexId : NeighborVertexSet)
				if (!vBoundaryStatus[NextVertexId])
					TutteMatrix.insert(VertexId, NextVertexId) = 1.0;
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
Eigen::VectorXd CArapParameterization::__solveSparseMatrix(const Eigen::SparseMatrix<double, Eigen::ColMajor>& vMatrix, const Eigen::VectorXd& vVector)
{
	auto CompressMatrix = vMatrix;
	CompressMatrix.makeCompressed();

	Eigen::SimplicialLDLT<Eigen::SparseMatrix<double, Eigen::ColMajor>> Solver;
	Solver.analyzePattern(CompressMatrix);
	Solver.factorize(CompressMatrix);
	_ASSERTE(Solver.info() == Eigen::Success);	//fixme: NumericalIssue
	auto Solution = Solver.solve(vVector);
	_ASSERTE(Solver.info() == Eigen::Success);
	auto Info = Solver.info();
	return Solution;
}

//*****************************************************************
//FUNCTION: 
Eigen::MatrixXd CArapParameterization::__switch2UVMatrix(const CMesh& vMesh, const Eigen::VectorXd& vX, const Eigen::VectorXd& vY)
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
//FUNCTION: 
int CArapParameterization::__findTwinRef(int vStartIndex, int vEndIndex)
{
	for(auto EdgeIndex : m_VertexInfoTable[vEndIndex])
		if (m_HalfEdgeTable[m_HalfEdgeTable[EdgeIndex]._Next]._VertexId == vStartIndex)
			return EdgeIndex;
	return -1;
}

//*****************************************************************
//FUNCTION: 
Eigen::MatrixXd CArapParameterization::__solveARAP(const Eigen::MatrixXd& vVertexPos, const Eigen::MatrixXi& vFaces, const Eigen::MatrixXd& vInitialUV)
{
	igl::ARAPData arap_data;
	arap_data.with_dynamics = true;
	Eigen::VectorXi Boundary = Eigen::VectorXi::Zero(0);
	Eigen::MatrixXd BoundaryCoord = Eigen::MatrixXd::Zero(0, 0);
	arap_data.max_iter = 100;

	// 2 means 2d
	igl::arap_precomputation(vVertexPos, vFaces, 2, Boundary, arap_data);
	auto UV = vInitialUV;
	igl::arap_solve(BoundaryCoord, arap_data, UV);
	
	return UV;
}