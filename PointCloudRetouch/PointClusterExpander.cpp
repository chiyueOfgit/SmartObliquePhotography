#include "pch.h"
#include "PointClusterExpander.h"
#include "common/CpuTimer.h"
#include "PointCluster.h"
#include "PointCloudRetouchManager.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_NORMAL_PRODUCT(CPointClusterExpander, KEYWORD::CLUSTER_EXPANDER)

//*****************************************************************
//FUNCTION: 
void CPointClusterExpander::runV(const CPointCluster* vCluster)
{
	if (vCluster == nullptr || vCluster->getCoreRegion().size() == 0)   //FIXME-014: 这里的参数有效性检查失败用抛异常的方式太过了
		_THROW_RUNTIME_ERROR("Expander input error");

	m_ExpandedPointSet.clear();

	CPointCloudRetouchManager *pManager = CPointCloudRetouchManager::getInstance();
	std::queue<pcl::index_t> ExpandingCandidateQueue = __initExpandingCandidateQueue(vCluster);
	std::deque TraversedFlag(pManager->getScene().getNumPoint(), false);  //FIXME-014：为什么用deque，感觉就是按vector在用
	
#ifdef _UNIT_TEST
	hiveCommon::CCPUTimer Timer;
	Timer.start();
#endif // _UNIT_TEST
	
	while (!ExpandingCandidateQueue.empty())
	{
		pcl::index_t Candidate = ExpandingCandidateQueue.front();
		ExpandingCandidateQueue.pop();

		if (TraversedFlag.at(Candidate))
			continue;
		else
			TraversedFlag.at(Candidate) = true;
		
		std::size_t CandidateLabel;  //FIXME-014：真是服了你，CandidateLabel故意要用std::size_t，在dumpPointLabelAt()里强制把EPointLabel转为size_t，拿到后，下面再强行转化回EPointLabel
		pManager->dumpPointLabelAt(CandidateLabel, Candidate);
		if (vCluster->getLabel() == EPointLabel::UNWANTED && static_cast<EPointLabel>(CandidateLabel) == EPointLabel::KEPT)
			continue;
		
		std::uint32_t OldClusterIndex = pManager->getClusterIndexAt(Candidate);
		//_ASSERTE(OldClusterIndex != vCluster->getClusterIndex());      //FIXME-014：故意注释这行代码让我来猜原因吗？

		double CurrentProbability = vCluster->evaluateProbability(Candidate);
		if (vCluster->isBelongingTo(CurrentProbability))
		{
			if (OldClusterIndex == 0 ||  //FIXME-014: 这里的0是什么意思？一个特殊的index吗？如果是，用#define来提高可读性
				__isReassigned2CurrentCluster(CurrentProbability, vCluster->getClusterIndex(), pManager->getClusterBelongingProbabilityAt(Candidate), OldClusterIndex))
			{
				if (static_cast<EPointLabel>(CandidateLabel) != EPointLabel::DISCARDED) pManager->tagPointLabel(Candidate, vCluster->getLabel(), vCluster->getClusterIndex(), CurrentProbability);
				m_ExpandedPointSet.push_back(Candidate);  //FIXME-014：这里的意思是，即使Candidate被标记为DISCARDED，它也会继续参与区域增长？
				for (auto e : pManager->buildNeighborhood(Candidate)) ExpandingCandidateQueue.push(e);  //FIXME-014：这里又需要建立邻域，整个过程buildNeighborhood调用过多少次？效率不考虑吗？
			}
		}
		else
		{//FIXME-014: else部分故意注释来玩的吗？
		//	//WARNING!
		//	pManager->tagPointLabel(Candidate, EPointLabel::FILLED, pManager->getClusterIndexAt(Candidate), pManager->getClusterBelongingProbabilityAt(Candidate));
			//pManager->tagPointLabel(Candidate, vCluster->getLabel(), vCluster->getClusterIndex(), CurrentProbability);
		//	//WARNING!
		//	hiveEventLogger::hiveOutputEvent(_FORMAT_STR2("Point: %1% is left in expander, its probability is %2%, below are infos:\n", Candidate, CurrentProbability) + vCluster->getDebugInfos(Candidate));
		}
	}

#ifdef _UNIT_TEST
	Timer.stop();
	m_RunTime = Timer.getElapsedTimeInMS();
#endif // _UNIT_TEST


	pManager->recordCurrentStatus();
}

//*****************************************************************
//FUNCTION: 
std::queue<pcl::index_t> CPointClusterExpander::__initExpandingCandidateQueue(const CPointCluster* vCluster)
{
	std::queue<pcl::index_t> CandidateQueue;
	for (auto Index : vCluster->getCoreRegion())
	{
		for (auto Neighbor : CPointCloudRetouchManager::getInstance()->buildNeighborhood(Index))  //FIXME-014：对每个core region里的点都去建邻域，不耗时吗 ？没有其他更好的方法了吗？哪怕是近似但效率更高的？
			if (find(vCluster->getCoreRegion().begin(), vCluster->getCoreRegion().end(), Neighbor) == vCluster->getCoreRegion().end())
				CandidateQueue.push(Neighbor);
	}
	//发生NRVO
	return CandidateQueue;
}

//*****************************************************************
//FUNCTION: 
bool CPointClusterExpander::__isReassigned2CurrentCluster(double vCurrentProbability, std::uint32_t vCurrentTimestamp, double vOldProbability, std::uint32_t vOldTimestamp)
{
	const double X = (vCurrentTimestamp - vOldTimestamp - 1.0) / 3.0;  //FIXME-014：一大堆magic number，这个函数的逻辑是什么？

	return vCurrentProbability > vOldProbability
	|| vCurrentTimestamp - vOldTimestamp > 4
	|| vCurrentProbability > vOldProbability * 2.0 * pow(X, 3.0) - 3.0 * pow(X, 2.0) + 1.0;
}