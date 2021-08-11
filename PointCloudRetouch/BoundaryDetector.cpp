#include "pch.h"
#include "BoundaryDetector.h"
#include "map"
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include "PointCloudRetouchManager.h"

#define PI 3.1415926
using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CBoundaryDetector, KEYWORD::BOUNDARY_DETECTOR)

bool CBoundaryDetector::init(const hiveConfig::CHiveConfig* vConfig)
{
	m_pBoundaryDetectorConfig = vConfig;
	return true;
}

//*****************************************************************
//FUNCTION: 
void CBoundaryDetector::runV(std::vector<pcl::index_t>& vioBoundarySet, std::vector<std::vector<pcl::index_t>>& voHoleSet)
{
	if (vioBoundarySet.empty())
		return;
	auto pManager = CPointCloudRetouchManager::getInstance();
	for (auto CurrentIndex : vioBoundarySet)
		if (CurrentIndex < 0 || CurrentIndex >= pManager->getRetouchScene().getNumPoint())
			_THROW_RUNTIME_ERROR("Index is out of range");

	std::vector<pcl::index_t> BoundarySet;

	std::mutex Mutex;
#pragma omp parallel for
	for(int m = 0;m < vioBoundarySet.size();m++)
	{
		auto Index = vioBoundarySet[m];
		auto HomoCenterPosition = pManager->getRetouchScene().getPositionAt(Index);
		auto HomoCenterNormal = pManager->getRetouchScene().getNormalAt(Index);
		Eigen::Vector3f CenterPosition{ HomoCenterPosition.x(), HomoCenterPosition.y(), HomoCenterPosition.z() };
		Eigen::Vector3f CenterNormal{ HomoCenterNormal.x(), HomoCenterNormal.y(), HomoCenterNormal.z() };
		CenterNormal.normalize();
		
		std::string SearchMode = m_pBoundaryDetectorConfig->getAttribute<std::string>("SEARCH_MODE_BOUNDARY_DETECTOR").value();
		int NearestK = m_pBoundaryDetectorConfig->getAttribute<int>("NEAREST_N_BOUNDARY_DETECTOR").value();;
		auto NeighborSet = pManager->buildNeighborhood(Index, SearchMode, NearestK);
		//FitNormalÔÝÊ±ÓÃCenterNormal´úÌæ
		Eigen::Vector3f FitNormal = CenterNormal;
		
		auto HomoStandardPos = pManager->getRetouchScene().getPositionAt(NeighborSet[1]);
		Eigen::Vector3f StandardPos{ HomoStandardPos.x(), HomoStandardPos.y(), HomoStandardPos.z() };
		auto StandardProjectivePos = __calcProjectivePoint(CenterPosition, FitNormal, StandardPos);
		Eigen::Vector3f StandardVector = StandardProjectivePos - CenterPosition;
		StandardVector.normalize();
		
		std::vector<float> AngleSet;
		int Sum = 0;
		for(int i = 1;i < NeighborSet.size();i++)
		{
			auto HomoNeighborPos = pManager->getRetouchScene().getPositionAt(NeighborSet[i]);
			if(pManager->getLabelAt(NeighborSet[i]) == EPointLabel::DISCARDED)
				continue;
			Eigen::Vector3f NeighborPos{ HomoNeighborPos.x(), HomoNeighborPos.y(), HomoNeighborPos.z() };
			auto ProjectivePos = __calcProjectivePoint(CenterPosition, FitNormal, NeighborPos);
			Eigen::Vector3f TempVector = ProjectivePos - CenterPosition;
			TempVector.normalize();
			auto Angle = __calcAngle(StandardVector, TempVector, FitNormal);
			AngleSet.push_back(Angle);
		}
		
		sort(AngleSet.begin(), AngleSet.end());
		int Size = AngleSet.size();
		if (2 * PI - AngleSet[Size - 1] > PI / 2 && 2 * PI - AngleSet[Size - 1] < PI)
			Sum++;
		else if(2 * PI - AngleSet[Size - 1] > PI)
			continue;
		for (int k = 1; k < Size; k++)
		{
			if (AngleSet[k] - AngleSet[k - 1] > PI / 2)
				Sum++;
			else if(AngleSet[k] - AngleSet[k - 1] > PI )
			{
				Sum = 0;
				break;
			}
		}
		if (Sum == 1)
		{
			Mutex.lock();
	        BoundarySet.push_back(Index);
			Mutex.unlock();
		}
	}

	auto TempSet = BoundarySet;
	vioBoundarySet.swap(TempSet) ;
	
	__divideBoundary(BoundarySet, voHoleSet);
	for (auto& Hole : voHoleSet)
	    for (auto Index : Hole)
			pManager->tagPointLabel(Index, EPointLabel::KEPT, 0, 0);
}

Eigen::Vector3f CBoundaryDetector::__calcProjectivePoint(Eigen::Vector3f& vCenterPosition, Eigen::Vector3f& vCenterNormal, Eigen::Vector3f& vProjectPosition)
{
	Eigen::Vector3f DiffVector = vProjectPosition - vCenterPosition;
	float Distance = DiffVector.dot(vCenterNormal);
	
	return vProjectPosition -  vCenterNormal * Distance;
}

float CBoundaryDetector::__calcAngle(Eigen::Vector3f& vStandardVector, Eigen::Vector3f& vOtherVector, Eigen::Vector3f& vCenterNormal)
{
	auto Dot = vStandardVector.dot(vOtherVector);
	if (Dot > 1.0f)
		Dot = 1.0f;
	else if (Dot < -1.0f)
		Dot = -1.0f;
	float Angle = std::acos(Dot);
	auto VectorCross = vStandardVector.cross(vOtherVector);
	VectorCross.normalize();
	if (VectorCross.dot(vCenterNormal) >= 0)
		return Angle;
	else
		return 2 * PI - Angle;
}

void CBoundaryDetector::__divideBoundary(std::vector<pcl::index_t>& vBoundaryPointSet, std::vector<std::vector<pcl::index_t>>& voHoleSet)
{
	if (vBoundaryPointSet.empty())
		return;

	std::vector<std::vector<pcl::index_t>> TempHoleSet;
	std::map<int, bool> TraversedFlag;
	for (auto Index : vBoundaryPointSet)
	{
		TraversedFlag.insert(std::make_pair(Index, false));
	}
	std::queue<pcl::index_t> ExpandingCandidateQueue;
	auto DistanceTolerance = m_pBoundaryDetectorConfig->getAttribute<float>("BOUNDARY_DISTANCE_TOLERANCE").value();
	
	while (!vBoundaryPointSet.empty())
	{
		ExpandingCandidateQueue.push(vBoundaryPointSet[0]);

		std::vector<pcl::index_t> TempBoundary;
		while (!ExpandingCandidateQueue.empty())
		{
			
			pcl::index_t Candidate = ExpandingCandidateQueue.front();
			ExpandingCandidateQueue.pop();
			
			TempBoundary.push_back(Candidate);
			
			std::vector<pcl::index_t> NeighborSet;
			__findNearestBoundaryPoint(Candidate, vBoundaryPointSet, NeighborSet, DistanceTolerance);
			if(NeighborSet.empty())
				continue;
			
			for(auto Index:NeighborSet)
			{
				if (TraversedFlag[Index] == true)
				    continue;
			    else
			    {
				    TraversedFlag[Index] = true;
				    ExpandingCandidateQueue.push(Index);
			    }
			}
		}
		if (TempBoundary.size() > m_pBoundaryDetectorConfig->getAttribute<int>("BOUNDARY_SIZE_TOLERANCE").value())
			TempHoleSet.push_back(TempBoundary);
		
		for (auto Iter = vBoundaryPointSet.begin(); Iter != vBoundaryPointSet.end(); )
		{
			if (TraversedFlag[*Iter] == true)
				Iter = vBoundaryPointSet.erase(Iter);
			else
				++Iter;
		}
		
	}

	int ClosedNum = 0;
	for (auto& Hole : TempHoleSet)
		if (__isClosedHoleBoundary(Hole))
			ClosedNum++;
	
	if(ClosedNum == 0)
	{
		std::vector<pcl::index_t> MergeSet;
		for (auto Temp : TempHoleSet)
			MergeSet.insert(MergeSet.end(), Temp.begin(), Temp.end());
		voHoleSet.push_back(MergeSet);
	}
	else
	{
		std::vector<pcl::index_t> MergeSet;
		for (auto Temp : TempHoleSet)
		{
			if (__isClosedHoleBoundary(Temp))
				voHoleSet.push_back(Temp);
			else
				MergeSet.insert(MergeSet.end(), Temp.begin(), Temp.end());
		}
		if (MergeSet.size() > 0.40f * vBoundaryPointSet.size())
			voHoleSet.push_back(MergeSet);
	}
}

void CBoundaryDetector::__findNearestBoundaryPoint(pcl::index_t vSeed, std::vector<pcl::index_t>& vTotalSet, std::vector<pcl::index_t>& voNeighborSet, float vTolerance)
{
	std::map<float, pcl::index_t> DistanceMap;
	auto pManager = CPointCloudRetouchManager::getInstance();
	auto SeedPos = pManager->getRetouchScene().getPositionAt(vSeed);
	for(auto Index: vTotalSet)
	{
		auto TempPos = pManager->getRetouchScene().getPositionAt(Index);
		if ((SeedPos - TempPos).norm() < vTolerance)
			DistanceMap.insert(std::make_pair((SeedPos - TempPos).norm(), Index));
	}
	for(auto Pair: DistanceMap)
		voNeighborSet.push_back(Pair.second);
}

bool CBoundaryDetector::__isClosedHoleBoundary(std::vector<pcl::index_t>& vHoleBoundary)
{
	auto DistanceTolerance = m_pBoundaryDetectorConfig->getAttribute<float>("BOUNDARY_IF_CLOSED_TOLERANCE").value();
	auto pManager = CPointCloudRetouchManager::getInstance();
	auto CurrentIndex = vHoleBoundary[0];
	std::vector<pcl::index_t> OrderedSet;
    OrderedSet.push_back(CurrentIndex);
	while (1)
	{
		std::vector<pcl::index_t> NeighborSet;
		__findNearestBoundaryPoint(CurrentIndex, vHoleBoundary, NeighborSet, DistanceTolerance);
		int i = 0;
		for(;i< NeighborSet.size();i++)
		{
			if(find(OrderedSet.begin(),OrderedSet.end(), NeighborSet[i])!= OrderedSet.end())
				continue;
			else
			{
				OrderedSet.push_back(NeighborSet[i]);
				CurrentIndex = NeighborSet[i];
				break;
			}
		}
		if (i == NeighborSet.size())
			break;
	}
	auto BeginPos = pManager->getRetouchScene().getPositionAt(OrderedSet[0]);
	auto EndPos = pManager->getRetouchScene().getPositionAt(OrderedSet[OrderedSet.size() - 1]);
	if ((BeginPos - EndPos).norm() <= DistanceTolerance)
		return true;
	else
		return false;
}