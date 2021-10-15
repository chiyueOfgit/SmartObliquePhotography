#include "pch.h"
#include "BasicMeshSuture.h"
#include <vcg/complex/algorithms/clean.h>
#include "FindSplitPlane.h"
#include "VcgMesh.hpp"

#include <fstream>	//remove
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/vector.hpp>

using namespace hiveObliquePhotography::SceneReconstruction;

_REGISTER_NORMAL_PRODUCT(CBasicMeshSuture, KEYWORD::BASIC_MESH_SUTURE)

//*****************************************************************
//FUNCTION: 
void CBasicMeshSuture::sutureMeshesV()
{
	_ASSERTE(m_SegmentPlane.norm());

	std::vector<int> LhsDissociatedIndices, RhsDissociatedIndices;
	std::vector<SVertex> LhsIntersectionPoints, RhsIntersectionPoints, PublicVertices;
	__executeIntersection(m_LhsMesh, m_SegmentPlane, LhsDissociatedIndices, LhsIntersectionPoints);
	__executeIntersection(m_RhsMesh, m_SegmentPlane, RhsDissociatedIndices, RhsIntersectionPoints);

	__serializeIndices(LhsDissociatedIndices, "Model_0_DissociatedPoints.txt");
	__serializeIndices(RhsDissociatedIndices, "Model_1_DissociatedPoints.txt");

	__generatePublicVertices(LHSIntersectionPoints, RHSIntersectionPoints, PublicVertices);
	m_MeshPlaneIntersection.sortPublic(PublicVertices);
	__connectVerticesWithMesh(m_LhsMesh, LHSDissociatedIndices, PublicVertices);
	__connectVerticesWithMesh(m_RhsMesh, RHSDissociatedIndices, PublicVertices);

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
	pcl::copyPointCloud(*vLhs, *SimpleLhs);
	pcl::copyPointCloud(*vRhs, *SimpleRhs);
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
void CBasicMeshSuture::__executeIntersection(CMesh& vioMesh, const Eigen::Vector4f& vPlane, std::vector<int>& voDissociatedIndices, std::vector<SVertex>& voIntersectionPoints)
{
	CMeshPlaneIntersection MeshPlaneIntersection;
	MeshPlaneIntersection.execute(vioMesh, vPlane);
	MeshPlaneIntersection.dumpDissociatedPoints(voDissociatedIndices);
	MeshPlaneIntersection.dumpIntersectionPoints(voIntersectionPoints);
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
void CBasicMeshSuture::__connectVerticesWithMesh(CMesh& vioMesh, std::vector<int>& vDissociatedIndices, std::vector<SVertex>& vPublicVertices)
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

	auto calcOrder = [&](const SFace& vFace) -> bool
	{
		const auto& A = vioMesh.m_Vertices[vFace.a];
		const auto& B = vioMesh.m_Vertices[vFace.b];
		const auto& C = vioMesh.m_Vertices[vFace.c];

		auto FaceNormal = (B.xyz() - A.xyz()).cross(C.xyz() - B.xyz()).normalized();
		auto AverageNormal = (A.normal() + B.normal() + C.normal()) / 3;
		return FaceNormal.dot(AverageNormal) > 0;
	};

	std::vector<SFace> IndexedFaceSet;

	bool ModelOrder;
	int NumTrue = 0, TestNum = 10;
	for (int i = 0; i < TestNum; i++)
		if (calcOrder(vioMesh.m_Faces[i]))
			NumTrue++;
	if ((float)NumTrue / TestNum >= 0.8)
		ModelOrder = true;
	else if ((float)NumTrue / TestNum <= 0.2)
		ModelOrder = false;
	else
		throw("Model error.");
	auto Order = true;
	do
	{
		IndexedFaceSet.clear();
		auto ConnectionFaceSet = __genConnectionFace(vDissociatedIndices.size(), PublicIndices.size(), true, Order);	// order is heuristic

		for (auto Offset = vDissociatedIndices.size(); auto & Face : ConnectionFaceSet)
		{
			SFace FaceWithMeshIndex;
			for (int i = 0; i < 3; i++)
				FaceWithMeshIndex[i] = Face[i] < Offset ? vDissociatedIndices[Face[i]] : PublicIndices[Face[i] - Offset];
			IndexedFaceSet.push_back(FaceWithMeshIndex);
		}

		Order = !Order;
	} while (calcOrder(IndexedFaceSet.front()) ^ calcOrder(vioMesh.m_Faces.front()));

	vioMesh.m_Faces.insert(vioMesh.m_Faces.end(), IndexedFaceSet.begin(), IndexedFaceSet.end());
}

//*****************************************************************
//FUNCTION: 
std::vector<hiveObliquePhotography::SFace> CBasicMeshSuture::__genConnectionFace(IndexType vNumLeft, IndexType vNumRight, bool vLeftBeforeRight, bool vIsClockwise)
{
	if (!vIsClockwise)
		return __genConnectionFace(vNumRight, vNumLeft, !vLeftBeforeRight, !vIsClockwise);

	std::vector<SFace> ConnectionFaceSet;
	std::pair<IndexType, IndexType> Offset(0, 0);
	if (vLeftBeforeRight)
		Offset.second = vNumLeft;
	else
		Offset.first = vNumRight;

	for (IndexType LeftCursor = 0, RightCursor = 0; LeftCursor < vNumLeft && RightCursor < vNumRight; )
	{
		auto LeftWithOffset = LeftCursor + Offset.first;
		auto RightWithOffset = RightCursor + Offset.second;

		if ((2 * LeftCursor + 1) * (vNumRight - 1) < (2 * RightCursor + 1) * (vNumLeft - 1))
		{
			if (LeftCursor + 1 >= vNumLeft)
				break;

			ConnectionFaceSet.emplace_back(LeftWithOffset, RightWithOffset, LeftWithOffset + 1);
			++LeftCursor;
		}
		else
		{
			if (RightCursor + 1 >= vNumRight)
				break;

			ConnectionFaceSet.emplace_back(LeftWithOffset, RightWithOffset, RightWithOffset + 1);
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
