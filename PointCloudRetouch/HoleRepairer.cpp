#include "pch.h"
#include "HoleRepairer.h"
#include "PlanarityFeature.h"
#include "PointCloudRetouchManager.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
void CHoleRepairer::repairHoleByBoundaryAndInput(const std::vector<pcl::index_t>& vBoundaryIndices, const std::vector<pcl::index_t>& vInputIndices, std::vector<pcl::PointSurfel>& voNewPoints, const hiveConfig::CHiveConfig* vConfig)
{
	const Eigen::Vector2i Resolution{ 32, 32 };

	//Boundary
	auto BoundaryPlane = __calculatePlaneByIndices(vBoundaryIndices);	//可以从别的地方给
	auto BoundaryBox = __calculateBoundingBoxByIndices(vBoundaryIndices);	//可以和indices无关
	SPlaneInfos BoundaryPlaneInfos;
	std::vector<std::vector<SLattice>> BoundaryPlaneLattices;
	__generatePlaneLattices(BoundaryPlane, BoundaryBox, Resolution, BoundaryPlaneInfos, BoundaryPlaneLattices);	//生成平面格子
	__projectPoints2PlaneLattices(vBoundaryIndices, BoundaryPlaneInfos, BoundaryPlaneLattices);	//投点进格子，未在里面的投不上

	//Input
	auto InputPlane = __calculatePlaneByIndices(vInputIndices);
	auto InputBox = __calculateBoundingBoxByIndices(vInputIndices);
	SPlaneInfos InputPlaneInfos; 
	std::vector<std::vector<SLattice>> InputPlaneLattices;
	__generatePlaneLattices(InputPlane, InputBox, Resolution, InputPlaneInfos, InputPlaneLattices);
	__projectPoints2PlaneLattices(vInputIndices, InputPlaneInfos, InputPlaneLattices);

	//生成颜色
	auto ColorLattices = __extractItemFromLattices<Eigen::Vector3i>(InputPlaneLattices, offsetof(SLattice, Color));
	//auto OutputColorLattices = CTextureGenerator::generateTexture<Eigen::Vector3i>(ColorLattices, Resolution);
	//__fillLatticesByItems<Eigen::Vector3i>(OutputColorLattices, BoundaryPlaneLattices, offsetof(SLattice, Color));

	//生成高度
	auto HeightLattices = __extractItemFromLattices<float>(InputPlaneLattices, offsetof(SLattice, Height));
	//auto OutputHeightLattices = CTextureGenerator::generateTexture<float>(HeightLattices, Resolution);
	//__fillLatticesByItems<float>(OutputHeightLattices, BoundaryPlaneLattices, offsetof(SLattice, Height));

	__generateNewPointsFromLattices(BoundaryPlane, BoundaryPlaneLattices, voNewPoints);
}

//*****************************************************************
//FUNCTION: 
void CHoleRepairer::__generatePlaneLattices(const Eigen::Vector4f& vPlane, const std::pair<Eigen::Vector3f, Eigen::Vector3f>& vBox, const Eigen::Vector2i& vResolution, SPlaneInfos& voPlaneInfos, std::vector<std::vector<SLattice>>& voPlaneLattices)
{
	std::vector<std::vector<SLattice>> PlaneLattices(vResolution.y(), std::vector<SLattice>(vResolution.x()));	//行优先

	Eigen::Vector3f PlaneNormal{ vPlane.x(), vPlane.y(), vPlane.z() };
	auto MinAxis = PlaneNormal.maxCoeff();
	std::vector<std::size_t> AxisOrder(3);	//Min在最后
	if (MinAxis == PlaneNormal.x())
		AxisOrder = { 1, 2, 0 };
	else if (MinAxis == PlaneNormal.y())
		AxisOrder = { 2, 0, 1 };
	else if (MinAxis == PlaneNormal.z())
		AxisOrder = { 0, 1, 2 };
	auto X = AxisOrder[0], Y = AxisOrder[1], Z = AxisOrder[2];

	Eigen::Vector3f BoxLength = vBox.second - vBox.first;
	float LatticeWidth = BoxLength.data()[X] / vResolution.x();
	float LatticeHeight = BoxLength.data()[Y] / vResolution.y();

	Eigen::Vector3f PlaneCenter;
	PlaneCenter.data()[X] = 0.5f * (vBox.first.data()[X] + vBox.second.data()[X]);
	PlaneCenter.data()[Y] = 0.5f * (vBox.first.data()[Y] + vBox.second.data()[Y]);
	PlaneCenter.data()[Z] = -(vPlane.w() + vPlane.data()[X] * PlaneCenter.data()[X] + vPlane.data()[Y] * PlaneCenter.data()[Y]) / vPlane.data()[Z];

	for (int Id = 0; Id < vResolution.x() * vResolution.y(); Id++)
	{
		Eigen::Vector2i LatticeCoord = { Id % vResolution.x(), int(Id / vResolution.x()) };
		Eigen::Vector3f LatticeWorldPos;
		LatticeWorldPos.data()[X] = vBox.first.data()[X] + (LatticeCoord.x() + 0.5f) * LatticeWidth;
		LatticeWorldPos.data()[Y] = vBox.first.data()[Y] + (LatticeCoord.y() + 0.5f) * LatticeHeight;
		LatticeWorldPos.data()[Z] = -(vPlane.w() + vPlane.data()[X] * LatticeWorldPos.data()[X] + vPlane.data()[Y] * LatticeWorldPos.data()[Y]) / vPlane.data()[Z];
		PlaneLattices[LatticeCoord.y()][LatticeCoord.x()].CenterPos = LatticeWorldPos;	//行优先
	}

	voPlaneInfos.AxisOrder = AxisOrder;
	voPlaneInfos.BoundingBox = vBox;
	voPlaneInfos.LatticeSize = { LatticeWidth, LatticeHeight };
	voPlaneInfos.Normal = PlaneNormal;
	voPlaneInfos.PlaneCenter = PlaneCenter;
	std::swap(voPlaneLattices, PlaneLattices);
}

//*****************************************************************
//FUNCTION: 
void CHoleRepairer::__projectPoints2PlaneLattices(const std::vector<pcl::index_t>& vIndices, const SPlaneInfos& vPlaneInfos, std::vector<std::vector<SLattice>>& vioPlaneLattices)
{
	_ASSERTE(!vioPlaneLattices.empty());
	auto Scene = CPointCloudRetouchManager::getInstance()->getRetouchScene();
	for (auto Index : vIndices)
	{
		auto Pos4f = Scene.getPositionAt(Index);
		Eigen::Vector3f PointPos{ Pos4f.x(), Pos4f.y(), Pos4f.z() };
		Eigen::Vector3f VecCenter2Point = PointPos - vPlaneInfos.PlaneCenter;
		Eigen::Vector3f VecProj2Point = VecCenter2Point.dot(vPlaneInfos.Normal) * vPlaneInfos.Normal;
		Eigen::Vector3f ProjPoint = vPlaneInfos.PlaneCenter + (VecCenter2Point - VecProj2Point);

		auto X = vPlaneInfos.AxisOrder[0], Y = vPlaneInfos.AxisOrder[1];
		Eigen::Vector2f Resolution = { vioPlaneLattices.front().size(), vioPlaneLattices.size() };
		Eigen::Vector2i LatticeCoord = { (ProjPoint.data()[X] - vPlaneInfos.BoundingBox.first.data()[X]) / vPlaneInfos.LatticeSize.x(), (ProjPoint.data()[Y] - vPlaneInfos.BoundingBox.first.data()[Y]) / vPlaneInfos.LatticeSize.y() };
		if (LatticeCoord.x() == Resolution.x())
			LatticeCoord.x() = Resolution.x() - 1;
		if (LatticeCoord.y() == Resolution.y())
			LatticeCoord.y() = Resolution.y() - 1;

		if (LatticeCoord.x() >= 0 && LatticeCoord.x() < Resolution.x() && LatticeCoord.y() >= 0 && LatticeCoord.y() < Resolution.y())
			vioPlaneLattices[LatticeCoord.y()][LatticeCoord.x()].Indices.push_back(Index);
	}
}

//*****************************************************************
//FUNCTION: 
void CHoleRepairer::__generateNewPointsFromLattices(const Eigen::Vector4f& vPlane, const std::vector<std::vector<SLattice>>& vPlaneLattices, std::vector<pcl::PointSurfel>& voNewPoints)
{
	_ASSERTE(!vPlaneLattices.empty());
	Eigen::Vector2i Resolution{ vPlaneLattices.front().size(), vPlaneLattices.size() };
	Eigen::Vector3f Normal = { vPlane.x(), vPlane.y(), vPlane.z() };

	const float K = 1.0f, B = 0.0f;	//线性系数

	std::vector<pcl::PointSurfel> NewPoints;
	for (int X = 0; X < Resolution.x(); X++)
	{
		for (int Y = 0; Y < Resolution.y(); Y++)
		{
			auto& Lattice = vPlaneLattices[Y][X];
			Eigen::Vector3f RealPos = Lattice.CenterPos + Normal * (K * Lattice.Height + B);	//取出加偏移
			pcl::PointSurfel TempPoint;
			TempPoint.x = RealPos.x();
			TempPoint.y = RealPos.y();
			TempPoint.z = RealPos.z();
			TempPoint.r = Lattice.Color.x();
			TempPoint.g = Lattice.Color.y();
			TempPoint.b = Lattice.Color.z();
			NewPoints.push_back(TempPoint);
		}
	}

	std::swap(voNewPoints, NewPoints);
}

Eigen::Vector4f CHoleRepairer::__calculatePlaneByIndices(const std::vector<pcl::index_t>& vIndices)
{
	_ASSERTE(!vIndices.empty());
	auto Scene = CPointCloudRetouchManager::getInstance()->getRetouchScene();
	pcl::PointCloud<pcl::PointXYZ>::Ptr BoundaryCloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (auto Index : vIndices)
	{
		pcl::PointXYZ TempPoint;
		auto Pos = Scene.getPositionAt(Index);
		TempPoint.x = Pos.x();
		TempPoint.y = Pos.y();
		TempPoint.z = Pos.z();
		BoundaryCloud->push_back(TempPoint);
	}

	return CPlanarityFeature::fitPlane(BoundaryCloud, 0.2f, { 0.0f, 0.0f, 1.0f });
}

std::pair<Eigen::Vector3f, Eigen::Vector3f> CHoleRepairer::__calculateBoundingBoxByIndices(const std::vector<pcl::index_t>& vIndices)
{
	_ASSERTE(!vIndices.empty());
	auto Scene = CPointCloudRetouchManager::getInstance()->getRetouchScene();
	Eigen::Vector3f Min{ FLT_MAX, FLT_MAX, FLT_MAX };
	Eigen::Vector3f Max{ -FLT_MAX, -FLT_MAX, -FLT_MAX };

	for (auto Index : vIndices)
	{
		auto Pos = Scene.getPositionAt(Index);
		for (int i = 0; i < 3; i++)
		{
			if (Pos.data()[i] < Min.data()[i])
				Min.data()[i] = Pos.data()[i];
			if (Pos.data()[i] > Max.data()[i])
				Max.data()[i] = Pos.data()[i];
		}
	}
	return { Min, Max };
}
