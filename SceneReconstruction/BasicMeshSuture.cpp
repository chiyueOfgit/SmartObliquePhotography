#include "pch.h"
#include "BasicMeshSuture.h"
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/vector.hpp>
#include <vcg/complex/algorithms/clean.h>
#include "FindSplitPlane.h"
#include "IntersectMeshAndPlane.h"
#include "VcgMesh.hpp"

using namespace hiveObliquePhotography::SceneReconstruction;

_REGISTER_NORMAL_PRODUCT(CBasicMeshSuture, KEYWORD::BASIC_MESH_SUTURE)

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::sutureMeshesV()
{
	_ASSERTE(m_SegmentPlane.norm());

	std::vector<SVertex> LhsIntersectionPoints, RhsIntersectionPoints, PublicVertices;
	std::vector<int> LhsDissociatedIndices, RhsDissociatedIndices;
	__executeIntersection(m_LhsMesh, m_SegmentPlane, LhsIntersectionPoints, LhsDissociatedIndices);
	__executeIntersection(m_RhsMesh, m_SegmentPlane, RhsIntersectionPoints, RhsDissociatedIndices);

	Eigen::Vector3f Direction;
	__findSutureDirection(m_LhsMesh, Direction);
	__sortDissociatedIndices(m_LhsMesh, LhsDissociatedIndices, Direction);
	__sortDissociatedIndices(m_RhsMesh, RhsDissociatedIndices, Direction);
	__sortIntersectionPoints(LhsIntersectionPoints, Direction);
	__sortIntersectionPoints(RhsIntersectionPoints, Direction);
	
	__serializeIndices(LhsDissociatedIndices, "Model_0_DissociatedPoints.txt");
	__serializeIndices(RhsDissociatedIndices, "Model_1_DissociatedPoints.txt");

	__generatePublicVertices(LhsIntersectionPoints, RhsIntersectionPoints, PublicVertices);
	__sortIntersectionPoints(PublicVertices, Direction);
	__connectVerticesWithMesh(m_LhsMesh, LhsDissociatedIndices, PublicVertices, Direction);
	__connectVerticesWithMesh(m_RhsMesh, RhsDissociatedIndices, PublicVertices, Direction);

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
void CBasicMeshSuture::__executeIntersection(CMesh& vioMesh, const Eigen::Vector4f& vPlane, std::vector<SVertex>& voIntersectionPoints, std::vector<int>& voDissociatedIndices)
{
	intersectMeshAndPlane(vioMesh, vPlane, voIntersectionPoints, voDissociatedIndices);
}

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::__generatePublicVertices(const std::vector<SVertex>& vLhs, const std::vector<SVertex>& vRhs, std::vector<SVertex>& voPublicVertices)
{
	voPublicVertices.clear();
	std::vector<SVertex> MajorPoints, MinorPoints, PairedMinorPoints;
	std::map<SVertex, SVertex> PairingPoints, PairingPointsAmended;
	bool Comparation = vLhs.size() > vRhs.size();
	MajorPoints = Comparation ? vLhs : vRhs;
	MinorPoints = Comparation ? vRhs : vLhs;

	for (const auto& MajorPoint : MajorPoints)
	{
		SVertex NearestPoint = __findNearestPoint(MinorPoints, MajorPoint);
		PairingPoints.insert(std::pair(MajorPoint, NearestPoint));
		PairedMinorPoints.push_back(NearestPoint);
	}

	for (const auto& MinorPoint : MinorPoints)
	{
		if (std::ranges::find(PairedMinorPoints, MinorPoint) != PairedMinorPoints.end())
			continue;

		SVertex NearestPoint = __findNearestPoint(MajorPoints, MinorPoint);
		PairingPointsAmended.insert(std::pair(NearestPoint, MinorPoint));
	}

	for (auto Iter = PairingPoints.begin(); Iter != PairingPoints.end(); ++Iter)
	{
		if (PairingPointsAmended.find(Iter->first) != PairingPointsAmended.end())
			voPublicVertices.push_back(lerp(
				lerp(Iter->first, Iter->second),
				lerp(Iter->first, PairingPointsAmended.find(Iter->first)->second)
			));
		else
			voPublicVertices.push_back(lerp(Iter->first, Iter->second));
	}
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
void CBasicMeshSuture::__connectVerticesWithMesh(CMesh& vioMesh, std::vector<int>& vDissociatedIndices, std::vector<SVertex>& vPublicVertices, const Eigen::Vector3f& vDirection)
{
	_ASSERTE(!vDissociatedIndices.empty() && !vPublicVertices.empty());

	std::vector<int> PublicIndices;
	PublicIndices.reserve(vPublicVertices.size());
	for (int i = vioMesh.m_Vertices.size(); const auto& Vertex : vPublicVertices)
	{
		vioMesh.m_Vertices.push_back(Vertex);
		PublicIndices.push_back(i++);
	}

	static int Count = 0;
	__serializeIndices(PublicIndices, "Model_" + std::to_string(Count++) + "_PublicPoints.txt");

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

		auto AB = B.xyz() - A.xyz();
		auto BC = C.xyz() - B.xyz();

		return AB.cross(BC).dot(Up) > 0;
	};

	bool ModelOrder = calcOrder(vioMesh.m_Faces.front());
	
	auto Order = true;
	std::vector<SFace> ConnectionFaceSet;
	do
	{
		Order = !Order;
		ConnectionFaceSet = __genConnectionFace(vioMesh, vDissociatedIndices, PublicIndices, vDirection, Order);	// order is heuristic

	} while (calcOrder(ConnectionFaceSet.front()) != ModelOrder);

	vioMesh.m_Faces.insert(vioMesh.m_Faces.end(), ConnectionFaceSet.begin(), ConnectionFaceSet.end());
}

//*****************************************************************
//FUNCTION: 
std::vector<hiveObliquePhotography::SFace> CBasicMeshSuture::__genConnectionFace(const CMesh& vMesh, const std::vector<int>& vLeft, const std::vector<int>& vRight, const Eigen::Vector3f& vDirection, bool vIsClockwise)
{
	if (!vIsClockwise)
		return __genConnectionFace(vMesh, vRight, vLeft, vDirection, !vIsClockwise);

	auto NumLeft = vLeft.size();
	auto NumRight = vRight.size();
	auto Direction = vDirection;

	auto calcDistance = [&](int vVertexIndex)
	{
		return vMesh.m_Vertices[vVertexIndex].xyz().dot(Direction);
	};
	if (calcDistance(vLeft.front()) > calcDistance(vLeft.back()))
		Direction = -Direction;

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
void CBasicMeshSuture::__serializeIndices(const std::vector<int>& vData, const std::string& vFileName) const
{
	std::ofstream Out(vFileName);
	boost::archive::text_oarchive Oarchive(Out);
	Oarchive& BOOST_SERIALIZATION_NVP(vData);
	Out.close();
}

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::__sortDissociatedIndices(const CMesh& vMesh, std::vector<int>& vioDissociatedPoints, Eigen::Vector3f& vDirection)
{
	std::vector<int> LocalOrderIndices;
	std::vector<int> OrderSet;
	auto compareV = [&](int vLhs, int vRhs) -> bool
	{
		return vMesh.m_Vertices[vLhs].xyz().dot(vDirection) < vMesh.m_Vertices[vRhs].xyz().dot(vDirection);
	};
	std::sort(vioDissociatedPoints.begin(), vioDissociatedPoints.end(), compareV);

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
void CBasicMeshSuture::__sortIntersectionPoints(std::vector<SVertex>& vioIntersectionPoints, Eigen::Vector3f& vDirection)
{
	std::vector<int> LocalOrderIndices;
	std::vector<SVertex> OrderSet;
	auto compareV = [&](const SVertex& vLhs, const SVertex& vRhs) -> bool
	{
		return vLhs.xyz().dot(vDirection) < vRhs.xyz().dot(vDirection);
	};
	std::sort(vioIntersectionPoints.begin(), vioIntersectionPoints.end(), compareV);
	__sortByVertexLoop(LocalOrderIndices, vioIntersectionPoints);
	for (auto Index : LocalOrderIndices)
		OrderSet.push_back(vioIntersectionPoints[Index]);
	vioIntersectionPoints.swap(OrderSet);
}

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::__sortByVertexLoop(std::vector<int>& vioOrderIndices, std::vector<SVertex>& vVertexSet)
{
	if (vVertexSet.empty())
		return;
	SVertex CurrentVertex = vVertexSet[0];
	std::vector<bool> Flag(vVertexSet.size(), false);
	auto Count = vVertexSet.size();
	while (Count > 0)
	{
		float MinDistance = FLT_MAX;
		int MinIndex;
		for (int i = 0; i < vVertexSet.size(); i++)
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
		Count--;
	}
}

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::__findSutureDirection(const CMesh& vMesh, Eigen::Vector3f& voDirection)
{
	std::pair<int, int> UV;
	int Height;
	vMesh.calcModelPlaneAxis(UV, Height);
	for (int i = 0; i < 3; i++)
	{
		if (i == Height)
			voDirection[i] = 1.0f;
		else
			voDirection[i] = 0.0f;
	}
	Eigen::Vector3f PlaneNormal{ m_SegmentPlane[0],m_SegmentPlane[1],m_SegmentPlane[2] };
	voDirection = voDirection.cross(PlaneNormal);
}