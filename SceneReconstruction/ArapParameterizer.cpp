#include "pch.h"
#include "ArapParameterizer.h"

#include <igl/arap.h>
#include <igl/readOBJ.h>
#include <igl/boundary_loop.h>

#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/vector.hpp>
#include <Eigen/IterativeLinearSolvers>

#include <vcg/complex/algorithms/clean.h>
#include "VcgMesh.hpp"

using namespace hiveObliquePhotography::SceneReconstruction;

_REGISTER_NORMAL_PRODUCT(CArapParameterizer, KEYWORD::ARAP_MESH_PARAMETERIZATION)

using namespace hiveObliquePhotography::SceneReconstruction;

//*****************************************************************
//FUNCTION: 
 bool CArapParameterizer::execute(Eigen::MatrixXd& voUV)
{
	//__executeProcessing(m_Mesh); //前处理暂时不做
 	buildHalfEdge();//根据m_mesh初始化m_HalfEdgeTable
	auto BoundarySet = findBoundaryPoint();//找到边界点的索引
	std::vector<bool> BoundaryStatus(m_Mesh.m_Vertices.size(), false);
	for (auto& Index : BoundarySet)//建立辅助vector，将所有边界点置为true
		BoundaryStatus[Index] = true;

	return calcInitialUV(m_Mesh, BoundaryStatus, voUV);
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
			m_VertexInfoTable[Face[i]].push_back(Index);//二维vector，存储着每个顶点在被多次访问的时候当前以建立半边的数目
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
//FUNCTION:
std::vector<int> CArapParameterizer::findBoundaryPoint()
{
 	//Eigen::MatrixXi F = m_Mesh.getFacesMatrix();//获取mesh所有顶点的坐标存为一个矩阵
	//std::vector<int> Boundary;
	//igl::boundary_loop(F, Boundary);//根据矩阵计算出所有边界点

	std::set<int> BoundarySet;
	std::vector<int> ValidSet;
	std::vector<int> FilteredSet;
	for(auto& HalfEdge : m_HalfEdgeTable)
	{
		if(HalfEdge._Conj < 0)
		{
			BoundarySet.insert(HalfEdge._VertexId);
			BoundarySet.insert(m_HalfEdgeTable[HalfEdge._Next]._VertexId);
		}
	}
	ValidSet.assign(BoundarySet.begin(), BoundarySet.end());
 	
	__filterBoundaryByGrid(ValidSet, 4, FilteredSet);
 	
	const std::string vPath = "Boundary25.txt";
	std::ofstream file(vPath.c_str());
	boost::archive::text_oarchive oa(file);
	oa& BOOST_SERIALIZATION_NVP(FilteredSet);
	file.close();
 	
	return FilteredSet;
}

//*****************************************************************
//FUNCTION: 
bool CArapParameterizer::calcInitialUV(const CMesh& vMesh, const std::vector<bool>& vBoundaryStatus, Eigen::MatrixXd& voUV)
{
	auto TutteMatrix = __buildTutteSolveMatrix(m_HalfEdgeTable, vBoundaryStatus);//计算映射矩阵，n*n
	Eigen::VectorXd VectorX, VectorY,AnswerX,AnswerY;
	//VectorX，VectorY用于固定边界坐标
	__fillTutteSolveVectors(VectorX, VectorY, vMesh, vBoundaryStatus);

	//解方程，这里也是调库
	if (!(__solveSparseMatrix(TutteMatrix, VectorX, AnswerX) && __solveSparseMatrix(TutteMatrix, VectorY, AnswerY)))
		return false;

	voUV= __switch2UVMatrix(vMesh, AnswerX, AnswerY);
	return true;
}

//*****************************************************************
//FUNCTION: 
Eigen::SparseMatrix<double, Eigen::ColMajor> CArapParameterizer::__buildTutteSolveMatrix(const std::vector<SHalfEdge>& vHalfEdgeSet, const std::vector<bool>& vBoundaryStatus)
{
	auto NumVertices = m_Mesh.m_Vertices.size();
	Eigen::SparseMatrix<double, Eigen::ColMajor> TutteMatrix(NumVertices, NumVertices);

	typedef Eigen::Triplet<double> TWeight;
	std::vector<TWeight> WeightTriplet;

	//在这里为什么要使用匿名函数呢？
	auto MaxNonZeroValueAmountOfRow = [&]()
	{
		int MaxNumber = 0;
		for (size_t VertexId = 0; VertexId < NumVertices; ++VertexId)
		{
			if (!vBoundaryStatus[VertexId]) //interior
			{
				if (MaxNumber < m_VertexInfoTable[VertexId].size())//代表这顶点被多少个mesh所共用
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
		{//边界点计算出两个对应的坐标存入
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
bool CArapParameterizer::__solveSparseMatrix(const Eigen::SparseMatrix<double, Eigen::ColMajor>& vMatrix, const Eigen::VectorXd& vVector, Eigen::VectorXd& voAnswer)
{
	auto CompressMatrix = vMatrix;
	CompressMatrix.makeCompressed();
	
	Eigen::BiCGSTAB<Eigen::SparseMatrix<double, Eigen::ColMajor>>Solver;
	Solver.analyzePattern(CompressMatrix);
	Solver.factorize(CompressMatrix);
	voAnswer = Solver.solve(vVector);

	switch (Solver.info())
	{
	case Eigen::Success:
		hiveEventLogger::hiveOutputEvent("Parameterization:Computation was successful.");
		return true;
		break;
	case Eigen::NumericalIssue:
		hiveEventLogger::hiveOutputEvent("Parameterization:The provided data did not satisfy the prerequisites.");
		return false;
		break;
	case Eigen::NoConvergence:
		hiveEventLogger::hiveOutputEvent("Parameterization:Iterative procedure did not converge.");
		return false;
		break;
	case Eigen::InvalidInput:
		hiveEventLogger::hiveOutputEvent("Parameterization:The inputs are invalid, or the algorithm has been improperly called.");
		return false;
		break;
	default:
		hiveEventLogger::hiveOutputEvent("Parameterization:Unexpected error.");
		return false;
		break;
	}
	
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

	//归一化到[0,1]
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

void CArapParameterizer::__filterBoundaryByGrid(std::vector<int>& vOriginSet, int vGridSize, std::vector<int>& vioFilteredSet)
 {
	auto Box = m_Mesh.calcAABB();
	int MinX = std::floor(Box.first.x()); int MaxX = std::ceil(Box.second.x());
	int MinY = std::floor(Box.first.y()); int MaxY = std::ceil(Box.second.y());

	std::vector<std::vector<std::vector<int>>> DistributionSet;
	DistributionSet.resize((MaxY - MinY) / vGridSize + 1, std::vector<std::vector<int>>((MaxX - MinX) / vGridSize + 1));
	for (auto Index: vOriginSet)
	{
		auto Pos = m_Mesh.m_Vertices[Index].xyz();
		DistributionSet[(Pos.y() - MinY) / vGridSize][(Pos.x() - MinX) / vGridSize].push_back(Index);
	}
	__filterOneDirection(DistributionSet, true, vioFilteredSet);
	__filterOneDirection(DistributionSet, false, vioFilteredSet);
 }

void CArapParameterizer::__filterOneDirection(std::vector<std::vector<std::vector<int>>>& vDistributionSet, bool vMainDirection, std::vector<int>& vioFilteredSet)
 {
	int MainSize = vMainDirection ? vDistributionSet.size() : vDistributionSet[0].size();
	int OtherSize = vMainDirection ? vDistributionSet[0].size() : vDistributionSet.size();
 	
 	for (int i = 0; i < MainSize; i++)
	{
		int j = 0;
		std::vector<int> TempMinSet;
		for (; j < OtherSize; j++)
		{
			TempMinSet = vMainDirection ? vDistributionSet[i][j] : vDistributionSet[j][i];
			if (TempMinSet.size())
				break;
		}
		if (j == OtherSize) continue;
		float Min = FLT_MAX;
		int MinFlag = 0;
		for (auto Index : TempMinSet)
		{
			auto MinTemp = vMainDirection ? m_Mesh.m_Vertices[Index].x : m_Mesh.m_Vertices[Index].y;
			if (MinTemp < Min)
			{
				Min = MinTemp;
				MinFlag = Index;
			}
		}
        vioFilteredSet.push_back(MinFlag);
 		
		int k = OtherSize - 1;
		std::vector<int> TempMaxSet;
		for (; k >= 0; k--)
		{
			TempMaxSet = vMainDirection ? vDistributionSet[i][k] : vDistributionSet[k][i];
			if (TempMaxSet.size())
				break;
		}
		float Max = -FLT_MAX;
		int MaxFlag = 0;
		for (auto Index : TempMaxSet)
		{
			auto MaxTemp = vMainDirection ? m_Mesh.m_Vertices[Index].x : m_Mesh.m_Vertices[Index].y;
			if (MaxTemp > Max)
			{
				Max = MaxTemp;
				MaxFlag = Index;
			}
		}
 		vioFilteredSet.push_back(MaxFlag);
	}
 }

void CArapParameterizer::__executeProcessing(CMesh& vioMesh)
 {
	CVcgMesh VcgMesh;
	toVcgMesh(vioMesh, VcgMesh);
	vcg::tri::Clean<CVcgMesh>::RemoveFaceOutOfRangeArea(VcgMesh, 0);
	//vcg::tri::Clean<CVcgMesh>::RemoveDuplicateVertex(VcgMesh);
	vcg::tri::Clean<CVcgMesh>::RemoveUnreferencedVertex(VcgMesh);
	vcg::tri::Allocator<CVcgMesh>::CompactFaceVector(VcgMesh);
	vcg::tri::Allocator<CVcgMesh>::CompactVertexVector(VcgMesh);
	fromVcgMesh(VcgMesh, vioMesh);
 }