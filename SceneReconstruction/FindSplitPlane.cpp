#include "pch.h"
#include "FindSplitPlane.h"
#include "PointCloudBoundingBox.hpp"

Eigen::Vector3f __findCentre(const std::pair<pcl::PointXYZ, pcl::PointXYZ>& vLhs, const std::pair<pcl::PointXYZ, pcl::PointXYZ>& vRhs);
Eigen::Vector3f __findMainAxis(const Eigen::Vector3f& vPositive, const Eigen::Vector3f& vNegative);

//*****************************************************************
//FUNCTION: 找到两个相邻点云模型之间的切割平面；
Eigen::Vector4f hiveObliquePhotography::SceneReconstruction::findSplitPlane(pcl::PointCloud<pcl::PointXYZ>::ConstPtr vLhs, pcl::PointCloud<pcl::PointXYZ>::ConstPtr vRhs)
{
	const auto AabbLhs = getAabb(vLhs);
	const auto AabbRhs = getAabb(vRhs);
	const auto Centre = __findCentre(AabbLhs, AabbRhs);
	const auto MainAxis = __findMainAxis(
		(AabbLhs.first.getVector3fMap() + AabbLhs.second.getVector3fMap()) / 2,
		(AabbRhs.first.getVector3fMap() + AabbRhs.second.getVector3fMap()) / 2
		);

	return { MainAxis.x(), MainAxis.y(), MainAxis.z(), -MainAxis.dot(Centre) };
}

//*****************************************************************
//FUNCTION: 
Eigen::Vector3f __findCentre(const std::pair<pcl::PointXYZ, pcl::PointXYZ>& vLhs, const std::pair<pcl::PointXYZ, pcl::PointXYZ>& vRhs)
{
	auto calcVolume = [](const pcl::PointXYZ& vLhs, const pcl::PointXYZ& vRhs)
	{
		Eigen::Vector3f DeltaPosition = vLhs.getVector3fMap() - vRhs.getVector3fMap();
		return abs(DeltaPosition.x() * DeltaPosition.y() * DeltaPosition.z());
	};

	std::pair<pcl::PointXYZ, pcl::PointXYZ> BoundingBox;
	if (calcVolume(vLhs.first, vRhs.second) < calcVolume(vRhs.first, vLhs.second))
		BoundingBox = { vLhs.first, vRhs.second };
	else
		BoundingBox = { vRhs.first, vLhs.second };
	
	return (BoundingBox.first.getVector3fMap() + BoundingBox.second.getVector3fMap()) / 2;
}

//*****************************************************************
//FUNCTION: 
Eigen::Vector3f __findMainAxis(const Eigen::Vector3f& vPositive, const Eigen::Vector3f& vNegative)
{
	const Eigen::Vector3f CandidateNormalSet[] =
	{
		{ 1, 0, 0 },
		{ 0, 1, 0 },
		{ 0, 0, 1 },
	};

	const Eigen::Vector3f* MainAxis = nullptr;
	float MaxDistance = -std::numeric_limits<float>::max();
	for (const auto& Candidate : CandidateNormalSet)
	{
		auto Distance = abs(Candidate.dot(vPositive - vNegative));
		if (MaxDistance < Distance)
		{
			MaxDistance = Distance;
			MainAxis = &Candidate;
		}
	}
	return *MainAxis;
}
