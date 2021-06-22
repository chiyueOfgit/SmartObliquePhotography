#include "pch.h"
#include "PointCluster.h"
#include "PointClusterExpander.h"
#include "PointCloudRetouchManager.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_NORMAL_PRODUCT(CPointClusterExpander, KEYWORD::CLUSTER_EXPANDER)

//*****************************************************************
//FUNCTION: 
void CPointClusterExpander::runV(const CPointCluster* vCluster)
{
	if (vCluster == nullptr)
		_THROW_RUNTIME_ERROR("Expander input error");

	CPointCloudRetouchManager *pManager = CPointCloudRetouchManager::getInstance();

	std::queue<pcl::index_t> ExpandingCandidateQueue = __initExpandingCandidateQueue(vCluster);

	while (!ExpandingCandidateQueue.empty())
	{
		pcl::index_t Candidate = ExpandingCandidateQueue.front();
		ExpandingCandidateQueue.pop();

		std::uint32_t OldClusterIndex = pManager->getClusterIndexAt(Candidate);
		_ASSERTE(OldClusterIndex != vCluster->getClusterIndex());

		double CurrentProbability = vCluster->evaluateProbability(Candidate);
		if (vCluster->isBelongingTo(CurrentProbability))
		{	
			if (OldClusterIndex == 0 ||
				__isReassigned2CurrentCluster(CurrentProbability, vCluster->getClusterIndex(), pManager->getClusterBelongingProbabilityAt(Candidate), OldClusterIndex))
			{
				pManager->tagPointLabel(Candidate, vCluster->getLabel(), vCluster->getClusterIndex(), CurrentProbability);

				for (auto e : pManager->buildNeighborhood(Candidate, vCluster->getClusterIndex()))
					ExpandingCandidateQueue.push(e);
			}
		}
	}
}

//*****************************************************************
//FUNCTION: 
std::queue<pcl::index_t> CPointClusterExpander::__initExpandingCandidateQueue(const CPointCluster* vCluster)
{
	std::queue<pcl::index_t> CandidateQueue;
	const auto SeedClusterIndex = vCluster->getClusterIndex();
	for(auto Index : vCluster->getCoreRegion())
	{
		for (auto Neighbor : CPointCloudRetouchManager::getInstance()->buildNeighborhood(Index, SeedClusterIndex))
			CandidateQueue.push(Neighbor);
	}
	//·¢ÉúNRVO
	return CandidateQueue;
}

//*****************************************************************
//FUNCTION: 
bool CPointClusterExpander::__isReassigned2CurrentCluster(double vCurrentProbability, std::uint32_t vCurrentTimestamp, double vOldProbability, std::uint32_t vOldTimestamp)
{
	return vCurrentProbability > vOldProbability || (vCurrentProbability > vOldProbability / 2 && vCurrentTimestamp - vOldTimestamp > 5);

}