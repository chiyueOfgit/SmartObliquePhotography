#include "pch.h"
#include "ArapParameterizer.h"

#include <igl/arap.h>
#include <igl/readOBJ.h>
#include <igl/boundary_loop.h>

#include <fstream>	//remove
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/vector.hpp>
#include <iomanip>
#include<Eigen/IterativeLinearSolvers>


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
	for (auto Index : BoundarySet)
		std::cout << "(" << InitialUV.row(Index).x() << " " << InitialUV.row(Index).y() << ") ";

	//auto UV = __solveARAP(m_Mesh.getVerticesMatrix(), m_Mesh.getFacesMatrix(), InitialUV, BoundarySet);
	return InitialUV;
}

//*****************************************************************
//FUNCTION: 
void CArapParameterizer::buildHalfEdge()
{
	m_VertexInfoTable.resize(m_Mesh.m_Vertices.size());
	m_HalfEdgeTable.clear();
	m_HalfEdgeTable.reserve(m_Mesh.m_Vertices.size() * 3);
	std::vector<bool> Flag(m_Mesh.m_Vertices.size(), false);
	int sum = 0;
	for(int FaceId = 0; FaceId < m_Mesh.m_Faces.size(); FaceId++)
	{
		auto& Face = m_Mesh.m_Faces[FaceId];
        Eigen::Vector3f A = m_Mesh.m_Vertices[Face[1]].xyz() - m_Mesh.m_Vertices[Face[0]].xyz();
		Eigen::Vector3f B = m_Mesh.m_Vertices[Face[2]].xyz() - m_Mesh.m_Vertices[Face[1]].xyz();
		Eigen::Vector3f Cross = A.cross(B);

		if (Cross.dot(m_Mesh.m_Vertices[Face[0]].normal()) < 0 || Cross.dot(m_Mesh.m_Vertices[Face[1]].normal()) < 0 || Cross.dot(m_Mesh.m_Vertices[Face[2]].normal()) < 0)
		{
			auto Temp = Face[1];
			Face[1] = Face[2];
			Face[2] = Temp;
		}

		for(int i = 0; i < 3; i++)
		{
			SHalfEdge HalfEdge;
			HalfEdge._VertexId = Face[i];
			HalfEdge._Face = FaceId;
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
std::vector<int> CArapParameterizer::findBoundaryPoint()
{
	//std::vector<bool> OutPutSet(m_Mesh.m_Vertices.size(), false);
	//std::set<int> BoundarySet;
	//std::vector<int> ValidSet;
	//for(auto& HalfEdge : m_HalfEdgeTable)
	//{
	//	if(HalfEdge._Conj < 0)
	//	{
	//		BoundarySet.insert(HalfEdge._VertexId);
	//		BoundarySet.insert(m_HalfEdgeTable[HalfEdge._Next]._VertexId);
	//	}
	//}

	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	igl::readOBJ(m_MeshPath, V, F);
	std::vector<int> Boundary;
	igl::boundary_loop(F, Boundary);
	//__findValidBoundary(Boundary, ValidSet);

	std::ofstream file("BoundaryPoints.txt");
	boost::archive::text_oarchive oa(file);
	oa& BOOST_SERIALIZATION_NVP(Boundary);
	file.close();

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
	TutteMatrix.reserve(Eigen::VectorXd::Zero(NumVertices * 3));

	auto Uniform = []()
	{
		return 1.0;
	};
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
		if (vBoundaryStatus[VertexId]) //boundary
			TutteMatrix.insert(VertexId, VertexId) = 1.0;
		else //interior
		{
			const auto& NeighborHalfEdgeSet = m_VertexInfoTable[VertexId];

			double SumWeight = 0;
			for (auto i : NeighborHalfEdgeSet)
			{
				auto NextVertexId = vHalfEdgeSet[vHalfEdgeSet[i]._Next]._VertexId;
				//auto Weight = Uniform();
				auto Weight = MeanWalue(i, VertexId, NextVertexId);
				TutteMatrix.insert(VertexId, NextVertexId) = Weight;
				SumWeight += Weight;
			}

			TutteMatrix.insert(VertexId, VertexId) = -1.0 * SumWeight;
		}
	}

	//cout matrix
	/*std::cout << "\n    Tutte: \n";
	for (int i = 0; i < TutteMatrix.cols(); i++)
	{
		for (int k = 0; k < TutteMatrix.rows(); k++)
			std::cout << std::setw(2) << TutteMatrix.coeff(k, i) << " ";
		std::cout << std::endl;
	}*/

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

	//Eigen::ConjugateGradient<Eigen::SparseMatrix<double, Eigen::ColMajor>>Solver;
	//Eigen::SimplicialLLT<Eigen::SparseMatrix<double, Eigen::ColMajor>> Solver;
	
	Eigen::BiCGSTAB<Eigen::SparseMatrix<double, Eigen::ColMajor>>Solver;
	Solver.analyzePattern(CompressMatrix);
	Solver.factorize(CompressMatrix);
	//_ASSERTE(Solver.info() == Eigen::Success);	//fixme: NumericalIssue
	auto Solution = Solver.solve(vVector);
	//_ASSERTE(Solver.info() == Eigen::Success);
	auto Info = Solver.info();

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
		//float U = vX(VertexId);
		//float V = vY(VertexId);

		UVMatrix.row(VertexId) = Eigen::Vector2d(U, V);
	}

	return UVMatrix;
}
//*****************************************************************
//FUNCTION: 
int CArapParameterizer::__findTwinRef(int vStartIndex, int vEndIndex)
{
	for(auto EdgeIndex : m_VertexInfoTable[vEndIndex])
		if (m_HalfEdgeTable[m_HalfEdgeTable[EdgeIndex]._Next]._VertexId == vStartIndex)
			return EdgeIndex;
	return -1;
}

//*****************************************************************
//FUNCTION: 
Eigen::MatrixXd CArapParameterizer::__solveARAP(const Eigen::MatrixXd& vVertexPos, const Eigen::MatrixXi& vFaces, const Eigen::MatrixXd& vInitialUV, const std::vector<int>& vBoundarySet)
{
	igl::ARAPData arap_data;
	arap_data.with_dynamics = false;
	int Size = vBoundarySet.size();
	Eigen::VectorXi Boundary(Size);
	Eigen::MatrixXd BoundaryCoord(Size, 2);
	arap_data.max_iter = 100;
	for (int i = 0; i < Size; i++)
	{
		Boundary[i] = vBoundarySet[i];
		BoundaryCoord.row(i) = vInitialUV.row(vBoundarySet[i]);
	}
	// 2 means 2d
	igl::arap_precomputation(vVertexPos, vFaces, 2, Boundary, arap_data);
	auto UV = vInitialUV;
	igl::arap_solve(BoundaryCoord, arap_data, UV);

	//__normalizeUV(UV);
	return UV;
}

void CArapParameterizer::__findValidBoundary(std::set<int>& vBoundarySet, std::vector<int>& voValidBoundary)
{
	int Sum = 0;
	auto Aabb = m_Mesh.calcAABB();
	auto Offset = (Aabb.second - Aabb.first) / 100.0;

	std::pair<int, int> XYAxis;
	int HeightAxis;
	m_Mesh.calcModelPlaneAxis(XYAxis, HeightAxis);
	
	for (auto& Boundary : vBoundarySet)
	{
		auto VetrexPosition = m_Mesh.m_Vertices[Boundary].xyz();
		auto MinX = std::min(abs(VetrexPosition.data()[XYAxis.first] - Aabb.second.data()[XYAxis.first]), abs(VetrexPosition.data()[XYAxis.first] - Aabb.first.data()[XYAxis.first]));
		auto MinY = std::min(abs(VetrexPosition.data()[XYAxis.second] - Aabb.second.data()[XYAxis.second]), abs(VetrexPosition.data()[XYAxis.second] - Aabb.first.data()[XYAxis.second]));
		if (MinX > Offset.x() && MinY > Offset.y())
			Sum++;
		else
			voValidBoundary.push_back(Boundary);
	}
}

void CArapParameterizer::__normalizeUV(Eigen::MatrixXd& vioUVMatrix)
{
	Eigen::Vector2d MinUV{ DBL_MAX, DBL_MAX };
	Eigen::Vector2d MaxUV{ -DBL_MAX, -DBL_MAX };
	auto update = [&](const Eigen::Vector2d& vUV)
	{
		for (int i = 0; i < 2; i++)
		{
			if (vUV.data()[i] < MinUV.data()[i])
				MinUV.data()[i] = vUV.data()[i];
			if (vUV.data()[i] > MaxUV.data()[i])
				MaxUV.data()[i] = vUV.data()[i];
		}
	};
	for (int VertexId = 0; VertexId < vioUVMatrix.rows(); VertexId++)
		update(vioUVMatrix.row(VertexId));

	auto WidthU = MaxUV[0] - MinUV[0];
	auto HeightV = MaxUV[1] - MinUV[1];

	for (int VertexId = 0; VertexId < vioUVMatrix.rows(); VertexId++)
	{
		float U = (vioUVMatrix.row(VertexId).data()[0] - MinUV[0]) / WidthU;
		float V = (vioUVMatrix.row(VertexId).data()[1] - MinUV[1]) / HeightV;
		vioUVMatrix.row(VertexId) = Eigen::Vector2d(U, V);
	}
}