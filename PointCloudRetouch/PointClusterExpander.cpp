#include "pch.h"
#include "PointCluster.h"
#include "PointClusterExpander.h"
#include "PointCloudRetouchManager.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
void CPointClusterExpander::runV(const CPointCluster* vCluster)
{
	_ASSERTE(vCluster);

	CPointCloudRetouchManager *pManager = CPointCloudRetouchManager::getInstance();

	std::queue<pcl::index_t> ExpandingCandidateQueue;

	__initExpandingCandidateQueue(vCluster, ExpandingCandidateQueue);

	std::vector<pcl::index_t> Neighborhood;
	while (!ExpandingCandidateQueue.empty())
	{
		pcl::index_t Candidate = ExpandingCandidateQueue.front();
		ExpandingCandidateQueue.pop();

		std::uint32_t OldClusterIndex = pManager->getClusterIndexAt(Candidate);
		_ASSERTE(OldClusterIndex != vCluster->getClusterIndex());

		bool IsClusterIndexRequired2Change = false;
		double CurrentProbability = vCluster->evaluateProbability(Candidate);
		if (vCluster->isBelongingTo(CurrentProbability))
		{
			IsClusterIndexRequired2Change = (OldClusterIndex == 0) ? true :
				__isReassigned2CurrentCluster(CurrentProbability, vCluster->getClusterIndex(), pManager->getClusterBelongingProbabilityAt(Candidate), OldClusterIndex);
		}

		if (IsClusterIndexRequired2Change)
		{
			pManager->tagPointLabel(Candidate, vCluster->getLabel(), vCluster->getClusterIndex(), CurrentProbability);
			pManager->buildNeighborhood(Candidate, vCluster->getClusterIndex(), Neighborhood);
			for (auto e : Neighborhood) ExpandingCandidateQueue.push(e);
		}
	}
}

//*****************************************************************
//FUNCTION: 
void CPointClusterExpander::__initExpandingCandidateQueue(const CPointCluster* vCluster, std::queue<pcl::index_t>& voCandidateQueue)
{

}

//*****************************************************************
//FUNCTION: 
bool CPointClusterExpander::__isReassigned2CurrentCluster(double vCurrentProbability, std::uint32_t vCurrentTimestamp, double vOldProbability, std::uint32_t vOldTimestamp)
{
//TODO��
	return true;
}