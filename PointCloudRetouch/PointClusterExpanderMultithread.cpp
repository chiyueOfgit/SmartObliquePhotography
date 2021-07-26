#include "pch.h"
#include "PointCluster.h"
#include "PointClusterExpanderMultithread.h"
#include "PointCloudRetouchManager.h"
#include "common/CpuTimer.h"
#include <tbb/parallel_for_each.h>

using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_NORMAL_PRODUCT(CPointClusterExpanderMultithread, KEYWORD::CLUSTER_EXPANDER_MULTITHREAD)

//*****************************************************************
//FUNCTION: 
void CPointClusterExpanderMultithread::runV(const CPointCluster* vCluster)
{
	if (vCluster == nullptr || vCluster->getCoreRegion().size() == 0)
		_THROW_RUNTIME_ERROR("Expander input error");

	m_ExpandPoints.clear();
	CPointCloudRetouchManager* pManager = CPointCloudRetouchManager::getInstance();
	auto ExpandingCandidateQueue = __initExpandingCandidateQueue(vCluster);
	std::vector<std::atomic_flag> TraversedFlag(pManager->getRetouchScene().getNumPoint());

	hiveCommon::CCPUTimer Timer;
	Timer.start();

	std::deque ExpandedFlag(pManager->getRetouchScene().getNumPoint(), false);
	tbb::parallel_for_each(ExpandingCandidateQueue.begin(), ExpandingCandidateQueue.end(),
		[&](pcl::index_t vCandidate, tbb::feeder<pcl::index_t>& vFeeder)
		{
			if (TraversedFlag.at(vCandidate).test_and_set())
				return;

			//std::size_t CandidateLabel;
			//pManager->dumpPointLabelAt(CandidateLabel, Candidate);
			//if (vCluster->getLabel() == EPointLabel::UNWANTED && static_cast<EPointLabel>(CandidateLabel) == EPointLabel::KEPT)
			//	continue;

			std::uint32_t OldClusterIndex = pManager->getClusterIndexAt(vCandidate);
			//_ASSERTE(OldClusterIndex != vCluster->getClusterIndex());

			double CurrentProbability = vCluster->evaluateProbability(vCandidate);
			if (vCluster->isBelongingTo(CurrentProbability))
			{
				if (OldClusterIndex == 0 ||
					__isReassigned2CurrentCluster(CurrentProbability, vCluster->getClusterIndex(), pManager->getClusterBelongingProbabilityAt(vCandidate), OldClusterIndex))
				{
					pManager->tagPointLabel(vCandidate, vCluster->getLabel(), vCluster->getClusterIndex(), CurrentProbability);
					ExpandedFlag.at(vCandidate) = true;

					for (auto e : pManager->buildNeighborhood(vCandidate))
						vFeeder.add(e);
				}
			}
			else
			{
				//	//WARNING!
				//	pManager->tagPointLabel(Candidate, EPointLabel::FILLED, pManager->getClusterIndexAt(Candidate), pManager->getClusterBelongingProbabilityAt(Candidate));
					//pManager->tagPointLabel(Candidate, vCluster->getLabel(), vCluster->getClusterIndex(), CurrentProbability);
				//	//WARNING!
				//	hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("Point: %1% is left in expander, its probability is %2%, below are infos:\n", Candidate, CurrentProbability) + vCluster->getDebugInfos(Candidate));
			}
		});
	for (size_t i = 0; i < ExpandedFlag.size(); i++)
	{
		if (ExpandedFlag.at(i))
			m_ExpandPoints.push_back(i);
	}
	
	Timer.stop();
	m_RunTime = Timer.getElapsedTimeInMS();

	pManager->recordCurrentStatus();
}

//*****************************************************************
//FUNCTION: 
std::vector<pcl::index_t> CPointClusterExpanderMultithread::__initExpandingCandidateQueue(const CPointCluster* vCluster)
{
	std::vector<pcl::index_t> CandidateQueue;
	for (auto Index : vCluster->getCoreRegion())
		for (auto Neighbor : CPointCloudRetouchManager::getInstance()->buildNeighborhood(Index))
			CandidateQueue.push_back(Neighbor);
	//·¢ÉúNRVO
	return CandidateQueue;
}

//*****************************************************************
//FUNCTION: 
bool CPointClusterExpanderMultithread::__isReassigned2CurrentCluster(double vCurrentProbability, std::uint32_t vCurrentTimestamp, double vOldProbability, std::uint32_t vOldTimestamp)
{
	const double X = (vCurrentTimestamp - vOldTimestamp - 1.0) / 3.0;

	return vCurrentProbability > vOldProbability
		|| vCurrentTimestamp - vOldTimestamp > 4
		|| vCurrentProbability > vOldProbability * 2.0 * pow(X, 3.0) - 3.0 * pow(X, 2.0) + 1.0;
}