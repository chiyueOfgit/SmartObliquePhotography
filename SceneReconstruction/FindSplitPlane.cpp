#include "pch.h"
#include "FindSplitPlane.h"
#include "PointCloudBoundingBox.hpp"

Eigen::Vector3f __findCentre(const std::pair<Eigen::Vector3f, Eigen::Vector3f>& vLhs, const std::pair<Eigen::Vector3f, Eigen::Vector3f>& vRhs);
Eigen::Vector3f __findMainAxis(const Eigen::Vector3f& vPositive, const Eigen::Vector3f& vNegative);

//*****************************************************************
//FUNCTION: 找到两个相邻点云模型之间的切割平面；
Eigen::Vector4f hiveObliquePhotography::SceneReconstruction::findSplitPlane(const CMesh& vLhs, const CMesh& vRhs)
{
	const auto AabbLhs = vLhs.calcAABB();
	const auto AabbRhs = vRhs.calcAABB();
	const auto Centre = __findCentre(AabbLhs, AabbRhs);
	const auto MainAxis = __findMainAxis(
		(AabbLhs.first + AabbLhs.second) / 2,
		(AabbRhs.first + AabbRhs.second) / 2
		);

	return { MainAxis.x(), MainAxis.y(), MainAxis.z(), -MainAxis.dot(Centre) };
}

//*****************************************************************
//FUNCTION: 
Eigen::Vector3f __findCentre(const std::pair<Eigen::Vector3f, Eigen::Vector3f>& vLhs, const std::pair<Eigen::Vector3f, Eigen::Vector3f>& vRhs)
{
	auto calcVolume = [](const Eigen::Vector3f& vLhs, const Eigen::Vector3f& vRhs)
	{
		Eigen::Vector3f DeltaPosition = vLhs - vRhs;
		return abs(DeltaPosition.x() * DeltaPosition.y() * DeltaPosition.z());
	};

	std::pair<Eigen::Vector3f, Eigen::Vector3f> BoundingBox;
	if (calcVolume(vLhs.first, vRhs.second) < calcVolume(vRhs.first, vLhs.second))
		BoundingBox = { vLhs.first, vRhs.second };
	else
		BoundingBox = { vRhs.first, vLhs.second };
	
	return (BoundingBox.first + BoundingBox.second) / 2;
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
