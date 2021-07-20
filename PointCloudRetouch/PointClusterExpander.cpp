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
	if (vCluster == nullptr || vCluster->getCoreRegion().size() == 0)
		_THROW_RUNTIME_ERROR("Expander input error");

	m_ExpandPoints.clear();

	CPointCloudRetouchManager *pManager = CPointCloudRetouchManager::getInstance();

	std::queue<pcl::index_t> ExpandingCandidateQueue = __initExpandingCandidateQueue(vCluster);

	while (!ExpandingCandidateQueue.empty())
	{
		pcl::index_t Candidate = ExpandingCandidateQueue.front();
		ExpandingCandidateQueue.pop();

		std::size_t CandidateLabel;
		pManager->dumpPointLabelAt(CandidateLabel, Candidate);
		if (vCluster->getLabel() == EPointLabel::UNWANTED && static_cast<EPointLabel>(CandidateLabel) == EPointLabel::KEPT)
			continue;
		
		std::uint32_t OldClusterIndex = pManager->getClusterIndexAt(Candidate);
		//_ASSERTE(OldClusterIndex != vCluster->getClusterIndex());

		double CurrentProbability = vCluster->evaluateProbability(Candidate);
		if (vCluster->isBelongingTo(CurrentProbability))
		{
			if (OldClusterIndex == 0 ||
				__isReassigned2CurrentCluster(CurrentProbability, vCluster->getClusterIndex(), pManager->getClusterBelongingProbabilityAt(Candidate), OldClusterIndex))
			{
				pManager->tagPointLabel(Candidate, vCluster->getLabel(), vCluster->getClusterIndex(), CurrentProbability);
				m_ExpandPoints.push_back(Candidate);

				for (auto e : pManager->buildNeighborhood(Candidate, vCluster->getClusterIndex()))
					ExpandingCandidateQueue.push(e);
			}
			
		}
		else
		{
		//	//WARNING!
		//	pManager->tagPointLabel(Candidate, EPointLabel::FILLED, pManager->getClusterIndexAt(Candidate), pManager->getClusterBelongingProbabilityAt(Candidate));
			pManager->tagPointLabel(Candidate, vCluster->getLabel(), vCluster->getClusterIndex(), CurrentProbability);
		//	//WARNING!
		//	hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("Point: %1% is left in expander, its probability is %2%, below are infos:\n", Candidate, CurrentProbability) + vCluster->getDebugInfos(Candidate));
		}
	}
	pManager->recordCurrentStatus();
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
	const double X = (vCurrentTimestamp - vOldTimestamp - 1.0) / 3.0;

	return vCurrentProbability > vOldProbability
	|| vCurrentTimestamp - vOldTimestamp > 4
	|| vCurrentProbability > vOldProbability * 2.0 * pow(X, 3.0) - 3.0 * pow(X, 2.0) + 1.0;
}