#include "pch.h"
#include "PointClusterExpanderMultithread4Auto.h"
#include <tbb/parallel_for_each.h>
#include "common/CpuTimer.h"
#include "PointCluster.h"
#include "PointCloudRetouchManager.h"
#include <fstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>

using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_NORMAL_PRODUCT(CPointClusterExpanderMultithread4Auto, KEYWORD::CLUSTER_EXPANDER_MULTITHREAD4AUTO)

//*****************************************************************
//FUNCTION: 
void CPointClusterExpanderMultithread4Auto::runV(const CPointCluster* vCluster)  //FIXME-014：这个函数copy/paste得很爽吧，连被注释的代码都一起copy/paste过来了
{
	if (vCluster == nullptr || vCluster->getCoreRegion().size() == 0)
		_THROW_RUNTIME_ERROR("Expander input error");

	m_ExpandedPointSet.clear();
	CPointCloudRetouchManager* pManager = CPointCloudRetouchManager::getInstance();
	

	std::vector<std::atomic_flag> TraversedFlag(pManager->getScene().getNumPoint());
	for (auto Index : vCluster->getCoreRegion())
	{
        TraversedFlag.at(Index).test_and_set();
		pManager->tagPointLabel(Index, vCluster->getLabel(), vCluster->getClusterIndex(), 1.0);
	}
		
	std::deque ExpandedFlag(pManager->getScene().getNumPoint(), false);

#ifdef _UNIT_TEST
	hiveCommon::CCPUTimer Timer;
	Timer.start();
#endif // _UNIT_TEST

	tbb::parallel_for_each(m_InitialCandidate.begin(), m_InitialCandidate.end(),
		[&](pcl::index_t vCandidate, tbb::feeder<pcl::index_t>& vFeeder)
		{
			if (TraversedFlag.at(vCandidate).test_and_set())
				return;

			std::size_t CandidateLabel;
			pManager->dumpPointLabelAt(CandidateLabel, vCandidate);
			if (vCluster->getLabel() == EPointLabel::UNWANTED && static_cast<EPointLabel>(CandidateLabel) == EPointLabel::KEPT)
				return;

			std::uint32_t OldClusterIndex = pManager->getClusterIndexAt(vCandidate);
			//_ASSERTE(OldClusterIndex != vCluster->getClusterIndex());

			double CurrentProbability = vCluster->evaluateProbability(vCandidate);
			if (vCluster->isBelongingTo(CurrentProbability))
			{
				if (OldClusterIndex == 0 ||
					__isReassigned2CurrentCluster(CurrentProbability, vCluster->getClusterIndex(), pManager->getClusterBelongingProbabilityAt(vCandidate), OldClusterIndex))
				{
					if (static_cast<EPointLabel>(CandidateLabel) != EPointLabel::DISCARDED) pManager->tagPointLabel(vCandidate, vCluster->getLabel(), vCluster->getClusterIndex(), CurrentProbability);
					ExpandedFlag.at(vCandidate) = true;
					for (auto e : pManager->buildNeighborhood(vCandidate)) vFeeder.add(e);
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
			m_ExpandedPointSet.push_back(i);
	}
	
#ifdef _UNIT_TEST
	Timer.stop();
	m_RunTime = Timer.getElapsedTimeInMS();
#endif // _UNIT_TEST


	pManager->recordCurrentStatus();
}

//*****************************************************************
//FUNCTION: 
std::vector<pcl::index_t> CPointClusterExpanderMultithread4Auto::__initExpandingCandidateQueue(const CPointCluster* vCluster)
{
	std::vector<pcl::index_t> CandidateQueue;
	for (auto Index : vCluster->getCoreRegion())
		for (auto Neighbor : CPointCloudRetouchManager::getInstance()->buildNeighborhood(Index))
			if(find(vCluster->getCoreRegion().begin(), vCluster->getCoreRegion().end(), Neighbor) == vCluster->getCoreRegion().end())
			CandidateQueue.push_back(Neighbor);
	//发生NRVO
	return CandidateQueue;
}

//*****************************************************************
//FUNCTION: 
bool CPointClusterExpanderMultithread4Auto::__isReassigned2CurrentCluster(double vCurrentProbability, std::uint32_t vCurrentTimestamp, double vOldProbability, std::uint32_t vOldTimestamp)
{
	const double X = (vCurrentTimestamp - vOldTimestamp - 1.0) / 3.0;

	return vCurrentProbability > vOldProbability
		|| vCurrentTimestamp - vOldTimestamp > 4
		|| vCurrentProbability > vOldProbability * 2.0 * pow(X, 3.0) - 3.0 * pow(X, 2.0) + 1.0;
}