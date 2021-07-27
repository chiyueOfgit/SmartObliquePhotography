#include "pch.h"
#include "BoundaryDetector.h"
#include "PointCloudRetouchManager.h"

#define PI 3.1415926
using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CBoundaryDetector, KEYWORD::BOUNDARY_DETECTOR)

//*****************************************************************
//FUNCTION: 
void CBoundaryDetector::runV(std::vector<pcl::index_t>& vioBoundarySet, const hiveConfig::CHiveConfig* vConfig)
{
	if (vioBoundarySet.empty())
		return;
	auto pManager = CPointCloudRetouchManager::getInstance();
	for (auto CurrentIndex : vioBoundarySet)
		if (CurrentIndex < 0 || CurrentIndex >= pManager->getRetouchScene().getNumPoint())
			_THROW_RUNTIME_ERROR("Index is out of range");

	std::vector<pcl::index_t> BoundarySet;
	for(auto Index: vioBoundarySet)
	{
		auto HomoCenterPosition = pManager->getRetouchScene().getPositionAt(Index);
		auto HomoCenterNormal = pManager->getRetouchScene().getNormalAt(Index);
		Eigen::Vector3f CenterPosition{ HomoCenterPosition.x(), HomoCenterPosition.y(), HomoCenterPosition.z() };
		Eigen::Vector3f CenterNormal{ HomoCenterNormal.x(), HomoCenterNormal.y(), HomoCenterNormal.z() };
		CenterNormal /= CenterNormal.norm();
		
		auto NeighborSet = pManager->buildNeighborhood(Index, 0);
		
		auto HomoStandardPos = pManager->getRetouchScene().getPositionAt(NeighborSet[1]);
		Eigen::Vector3f StandardPos{ HomoStandardPos.x(), HomoStandardPos.y(), HomoStandardPos.z() };
		auto StandardProjectivePos = __calcProjectivePoint(CenterPosition, CenterNormal, StandardPos);
		Eigen::Vector3f StandardVector = (StandardProjectivePos - CenterPosition) / (StandardProjectivePos - CenterPosition).norm();
		
		std::vector<float> Quadrant;
		int Sum = 0;
		for(int i = 1;i < NeighborSet.size();i++)
		{
			auto HomoNeighborPos = pManager->getRetouchScene().getPositionAt(NeighborSet[i]);
			Eigen::Vector3f NeighborPos{ HomoNeighborPos.x(), HomoNeighborPos.y(), HomoNeighborPos.z() };
			auto ProjectivePos = __calcProjectivePoint(CenterPosition, CenterNormal, NeighborPos);
			Eigen::Vector3f TempVector = (ProjectivePos - CenterPosition) / (ProjectivePos - CenterPosition).norm();
			auto Angle = __calcAngle(StandardVector, TempVector, CenterNormal);
			Quadrant.push_back(Angle);
		}
		sort(Quadrant.begin(), Quadrant.end());
		for(int k = 1;k< Quadrant.size();k++)
			if (Quadrant[k] - Quadrant[k-1] > PI / 2)
				Sum++;
		if (2 * PI - Quadrant[Quadrant.size() - 1] > PI / 2 && 2 * PI - Quadrant[Quadrant.size() - 1] < PI)
			Sum++;
		if (Sum == 1)
		{
			BoundarySet.push_back(Index);
			pManager->tagPointLabel(Index, EPointLabel::UNWANTED, 0, 0);
		}
	}
}

Eigen::Vector3f CBoundaryDetector::__calcProjectivePoint(Eigen::Vector3f& vCenterPosition, Eigen::Vector3f& vCenterNormal, Eigen::Vector3f& vProjectPosition)
{
	Eigen::Vector3f DiffVector = vProjectPosition - vCenterPosition;
	float Distance = DiffVector.norm();
	float CosAngle = DiffVector.dot(vCenterNormal);
	
	return vProjectPosition -  vCenterNormal * CosAngle;
}

float CBoundaryDetector::__calcAngle(Eigen::Vector3f& vStandardVector, Eigen::Vector3f& vOtherVector, Eigen::Vector3f& vCenterNormal)
{
	auto Dot = vStandardVector.dot(vOtherVector);
	if (Dot > 1.0f)
		Dot = 1.0f;
	else if (Dot < -1.0f)
		Dot = -1.0f;
	float Angle = std::acos(Dot);
	auto b = (vStandardVector.cross(vOtherVector)).dot(vCenterNormal);
	if ((vStandardVector.cross(vOtherVector)).dot(vCenterNormal) >= 0)
		return Angle;
	else
		return 2 * PI - Angle;
}