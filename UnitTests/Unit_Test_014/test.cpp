#include "pch.h"
#include "PointCloudRetouchInterface.h"
#include "PointCluster.h"
#include "PointClusterExpander.h"

//测试用例列表：
//  * DeathTest_EmptyInput:尝试输入空的集合；
//  * DeathTest_NullptrInput:尝试输入空集合指针；
//	* No_RepeatIndex_Test: 生成的初始CandidateQueue中不应该有输入的Cluster的索引

using namespace  hiveObliquePhotography::PointCloudRetouch;

TEST(DeathTest_EmptyInput, EmptyInput)
{
	CPointClusterExpander* pPointClusterExpander;
	CPointCluster* UserSpecifiedCluster;
	
	ASSERT_ANY_THROW(pPointClusterExpander->execute<CPointClusterExpander>(UserSpecifiedCluster));
}

TEST(DeathTest_NullptrInput, NullptrInput)
{
	CPointClusterExpander* pPointClusterExpander;
	CPointCluster* UserSpecifiedCluster = nullptr;

	ASSERT_ANY_THROW(pPointClusterExpander->execute<CPointClusterExpander>(UserSpecifiedCluster));
}

TEST(No_RepeatIndex_Test, NoRepeatIndex)
{
	CPointClusterExpander* pPointClusterExpander;
	CPointCluster* UserSpecifiedCluster;

	std::queue<pcl::index_t> CandidateQueue;
	pPointClusterExpander->initExpandingCandidateQueue(UserSpecifiedCluster, CandidateQueue);
	int Sum = 0;
	while(!CandidateQueue.empty())
	{
		pcl::index_t Index = CandidateQueue.front();
		CandidateQueue.pop();
		if (find(UserSpecifiedCluster->getCoreRegion().begin(), UserSpecifiedCluster->getCoreRegion().end(), Index) != UserSpecifiedCluster->getCoreRegion().end())
			Sum++;
	}
	ASSERT_EQ(Sum, 0);
}