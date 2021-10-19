#include "pch.h"
#include "BasicMeshSuture.h"
#include <boost/serialization/vector.hpp>
#include <vcg/complex/algorithms/clean.h>
#include "FindSplitPlane.h"
#include "IntersectMeshAndPlane.h"
#include "VcgMesh.hpp"

using namespace hiveObliquePhotography::SceneReconstruction;
using hiveObliquePhotography::SVertex;
using hiveObliquePhotography::SFace;

_REGISTER_NORMAL_PRODUCT(CBasicMeshSuture, KEYWORD::BASIC_MESH_SUTURE)

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::sutureMeshesV()
{
	_ASSERTE(!m_SegmentPlane.isZero());

	auto [LhsIntersectionPoints, LhsDissociatedIndices] = intersectMeshAndPlane(m_SegmentPlane, m_LhsMesh);
	auto [RhsIntersectionPoints, RhsDissociatedIndices] = intersectMeshAndPlane(m_SegmentPlane, m_RhsMesh);
	m_Direction = __calcSutureDirection(m_LhsMesh);
	
	__sortDissociatedIndices(m_LhsMesh, LhsDissociatedIndices);
	__sortDissociatedIndices(m_RhsMesh, RhsDissociatedIndices);
	__sortIntersectionPoints(LhsIntersectionPoints);
	__sortIntersectionPoints(RhsIntersectionPoints);

#ifdef _DEBUG
	__serializeIndices(LhsDissociatedIndices, "Model_0_DissociatedPoints.txt");
	__serializeIndices(RhsDissociatedIndices, "Model_1_DissociatedPoints.txt");
#endif

	std::vector<SVertex> PublicVertices = __generatePublicVertices(LhsIntersectionPoints, RhsIntersectionPoints);
	__sortIntersectionPoints(PublicVertices);
	__connectVerticesWithMesh(LhsDissociatedIndices, PublicVertices, m_LhsMesh);
	__connectVerticesWithMesh(RhsDissociatedIndices, PublicVertices, m_RhsMesh);

	__removeUnreferencedVertex(m_LhsMesh);
	__removeUnreferencedVertex(m_RhsMesh);
}

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::setCloud4SegmentPlane(PointCloud_t::ConstPtr vLhs, PointCloud_t::ConstPtr vRhs)
{
	using SimpleCloudType = pcl::PointCloud<pcl::PointXYZ>;
	
	SimpleCloudType::Ptr SimpleLhs(new SimpleCloudType);
	SimpleCloudType::Ptr SimpleRhs(new SimpleCloudType);
	copyPointCloud(*vLhs, *SimpleLhs);
	copyPointCloud(*vRhs, *SimpleRhs);
	m_SegmentPlane = findSplitPlane(SimpleLhs, SimpleRhs);
}

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::dumpMeshes(CMesh& voLhsMesh, CMesh& voRhsMesh) const
{
	voLhsMesh = m_LhsMesh;
	voRhsMesh = m_RhsMesh;
}

//*****************************************************************
//FUNCTION: 
std::vector<SVertex> CBasicMeshSuture::__generatePublicVertices(const std::vector<SVertex>& vLhs, const std::vector<SVertex>& vRhs)
{
	std::vector<SVertex> PublicVertices;
	
	const std::vector<SVertex> *pMajorPoints = &vLhs, *pMinorPoints = &vRhs;
	if (vLhs.size() < vRhs.size())
		std::swap(pMajorPoints, pMinorPoints);
	std::vector<SVertex> PairedMinorPoints;
	std::map<SVertex, SVertex> PairingPoints, PairingPointsAmended;

	for (const auto& MajorPoint : *pMajorPoints)
	{
		SVertex NearestPoint = __findNearestPoint(*pMinorPoints, MajorPoint);
		PairingPoints.insert(std::pair(MajorPoint, NearestPoint));
		PairedMinorPoints.push_back(NearestPoint);
	}

	for (const auto& MinorPoint : *pMinorPoints)
	{
		if (std::ranges::find(PairedMinorPoints, MinorPoint) != PairedMinorPoints.end())
			continue;

		SVertex NearestPoint = __findNearestPoint(*pMajorPoints, MinorPoint);
		PairingPointsAmended.insert(std::pair(NearestPoint, MinorPoint));
	}

	for (auto Iter = PairingPoints.begin(); Iter != PairingPoints.end(); ++Iter)
	{
		if (PairingPointsAmended.find(Iter->first) != PairingPointsAmended.end())
			PublicVertices.push_back(lerp(
				lerp(Iter->first, Iter->second),
				lerp(Iter->first, PairingPointsAmended.find(Iter->first)->second)
			));
		else
			PublicVertices.push_back(lerp(Iter->first, Iter->second));
	}
	return PublicVertices;
}

//*****************************************************************
//FUNCTION: 
auto CBasicMeshSuture::__findNearestPoint(const std::vector<SVertex>& vVectexSet, const SVertex& vOrigin) -> SVertex
{
	auto MinDistance = std::numeric_limits<decltype(vOrigin.xyz())::value_type>::max();
	auto Nearest = vVectexSet.begin();
	for (auto i = vVectexSet.begin(); i != vVectexSet.end(); ++i)
	{
		auto Distance = (vOrigin.xyz() - i->xyz()).norm();
		if (MinDistance > Distance)
		{
			MinDistance = Distance;
			Nearest = i;
		}
	}
	return *Nearest;
}

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::__connectVerticesWithMesh(const std::vector<int>& vDissociatedIndices, const std::vector<SVertex>& vPublicVertices, CMesh& vioMesh)
{
	_ASSERTE(!vDissociatedIndices.empty() && !vPublicVertices.empty());

	std::vector<int> PublicIndices;
	PublicIndices.reserve(vPublicVertices.size());
	for (int i = vioMesh.m_Vertices.size(); const auto& Vertex : vPublicVertices)
	{
		vioMesh.m_Vertices.push_back(Vertex);
		PublicIndices.push_back(i++);
	}

#ifdef _DEBUG
	static int Count = 0;
	__serializeIndices(PublicIndices, "Model_" + std::to_string(Count++) + "_PublicPoints.txt");
#endif

	int Height;
	std::pair<int, int> UV;
	vioMesh.calcModelPlaneAxis(UV, Height);
	Eigen::Vector3f Up = { 0.0f, 0.0f, 0.0f };
	Up.data()[Height] = 1.0f;
	auto calcOrder = [&](const SFace& vFace) -> bool
	{
		const auto& A = vioMesh.m_Vertices[vFace.a];
		const auto& B = vioMesh.m_Vertices[vFace.b];
		const auto& C = vioMesh.m_Vertices[vFace.c];

		return (B.xyz() - A.xyz()).cross(C.xyz() - B.xyz()).dot(Up) > 0;
	};

	bool ModelOrder = calcOrder(vioMesh.m_Faces.front());
	auto Order = true;
	std::vector<SFace> ConnectionFaceSet;
	do
	{
		Order = !Order;
		ConnectionFaceSet = __genConnectionFace(vioMesh, vDissociatedIndices, PublicIndices, Order);	// order is heuristic

	} while (calcOrder(ConnectionFaceSet.front()) != ModelOrder);

	vioMesh.m_Faces.insert(vioMesh.m_Faces.end(), ConnectionFaceSet.begin(), ConnectionFaceSet.end());
}

//*****************************************************************
//FUNCTION: 
std::vector<SFace> CBasicMeshSuture::__genConnectionFace(const CMesh& vMesh, const std::vector<int>& vLeft, const std::vector<int>& vRight, bool vIsClockwise)
{
	if (!vIsClockwise)
		return __genConnectionFace(vMesh, vRight, vLeft, !vIsClockwise);

	auto NumLeft = vLeft.size(), NumRight = vRight.size();
	auto SignedDirection = m_Direction;

	auto calcDistance = [&](int vVertexIndex)
	{
		return vMesh.m_Vertices[vVertexIndex].xyz().dot(SignedDirection);
	};
	if (calcDistance(vLeft.front()) > calcDistance(vLeft.back()))
		SignedDirection = -SignedDirection;

	std::vector<SFace> ConnectionFaceSet;
	for (IndexType LeftCursor = 0, RightCursor = 0; LeftCursor < NumLeft && RightCursor < NumRight; )
	{
		if (calcDistance(vLeft[LeftCursor]) <= calcDistance(vRight[RightCursor]))
		{
			if (LeftCursor + 1 >= NumLeft)
				break;

			ConnectionFaceSet.emplace_back(vLeft[LeftCursor], vRight[RightCursor], vLeft[LeftCursor + 1]);
			++LeftCursor;
		}
		else
		{
			if (RightCursor + 1 >= NumRight)
				break;

			ConnectionFaceSet.emplace_back(vLeft[LeftCursor], vRight[RightCursor], vRight[RightCursor + 1]);
			++RightCursor;
		}
	}
	return ConnectionFaceSet;
}

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::__removeUnreferencedVertex(CMesh& vioMesh)
{
	CVcgMesh VcgMesh;
	toVcgMesh(vioMesh, VcgMesh);
	vcg::tri::Clean<CVcgMesh>::RemoveUnreferencedVertex(VcgMesh);
	fromVcgMesh(VcgMesh, vioMesh);
}

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::__sortDissociatedIndices(const CMesh& vMesh, std::vector<int>& vioDissociatedPoints)
{
	std::vector<int> LocalOrderIndices;
	std::vector<int> OrderSet;
	auto compareV = [&](int vLhs, int vRhs) -> bool
	{
		return vMesh.m_Vertices[vLhs].xyz().dot(m_Direction) < vMesh.m_Vertices[vRhs].xyz().dot(m_Direction);
	};
	std::ranges::sort(vioDissociatedPoints, compareV);

	std::vector<SVertex> VertexSet;
	for (auto Index : vioDissociatedPoints)
		VertexSet.push_back(vMesh.m_Vertices[Index]);
	__sortByVertexLoop(LocalOrderIndices, VertexSet);
	for(auto Index : LocalOrderIndices)
		OrderSet.push_back(vioDissociatedPoints[Index]);
	vioDissociatedPoints.swap(OrderSet);
}

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::__sortIntersectionPoints(std::vector<SVertex>& vioIntersectionPoints)
{
	std::vector<int> LocalOrderIndices;
	std::vector<SVertex> OrderSet;
	auto compareV = [this](const SVertex& vLhs, const SVertex& vRhs) -> bool
	{
		return vLhs.xyz().dot(m_Direction) < vRhs.xyz().dot(m_Direction);
	};
	std::ranges::sort(vioIntersectionPoints, compareV);
	__sortByVertexLoop(LocalOrderIndices, vioIntersectionPoints);
	for (auto Index : LocalOrderIndices)
		OrderSet.push_back(vioIntersectionPoints[Index]);
	vioIntersectionPoints.swap(OrderSet);
}

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::__sortByVertexLoop(std::vector<int>& vioOrderIndices, const std::vector<SVertex>& vVertexSet)
{
	if (vVertexSet.empty())
		return;
	SVertex CurrentVertex = vVertexSet[0];
	std::deque Flag(vVertexSet.size(), false);
	auto Count = vVertexSet.size();
	while (Count > 0)
	{
		float MinDistance = FLT_MAX;
		int MinIndex;
		for (size_t i = 0; i < vVertexSet.size(); ++i)
		{
			if (!Flag[i])
			{
				auto TempDis = (CurrentVertex.xyz() - vVertexSet[i].xyz()).norm();
				if (TempDis < MinDistance)
				{
					MinDistance = TempDis;
					MinIndex = i;
				}
			}
		}
		if (MinDistance > 30)
			break;
		vioOrderIndices.push_back(MinIndex);
		CurrentVertex = vVertexSet[MinIndex];
		Flag[MinIndex] = true;
		--Count;
	}
}

//*****************************************************************
//FUNCTION: 
Eigen::Vector3f CBasicMeshSuture::__calcSutureDirection(const CMesh& vMesh)
{
	std::pair<int, int> UV; int Height;
	vMesh.calcModelPlaneAxis(UV, Height);

	Eigen::Vector3f Direction = Eigen::Vector3f::Zero();
	Direction.data()[Height] = 1.0f;

	return Direction.cross(Eigen::Vector3f(m_SegmentPlane[0], m_SegmentPlane[1], m_SegmentPlane[2]));
}