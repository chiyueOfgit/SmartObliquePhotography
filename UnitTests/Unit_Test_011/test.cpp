#include "pch.h"
#include "PointCloudRetouchInterface.h"
#include "InitialClusterCreator.h"
//主要围绕CInitialClusterCreator::createInitialCluster()进行测试

//测试用例列表：
//  * GenerateHardness4EveryPoint: 能够以期望方式生成硬度
//  * ComputeClusterCenter: 能够正确算出中心点
//  * DivideUserSpecifiedRegion: 能够以期望方式划分区域
//  * InitPointCluster: 能够正常初始化Cluster
// 
//  * CreateInitialCluster: 能够正确创建初始簇

using namespace hiveObliquePhotography::PointCloudRetouch;

class CTestCreateInitialCluster : public testing::Test
{
protected:
	void SetUp() override
	{
		m_pCloud1.reset(new PointCloud_t);
		m_pCloud1->resize(100);


	}

	void TearDown() override
	{

	}

	PointCloud_t::Ptr m_pCloud1 = nullptr;
};

TEST(Test_CreateInitialCluster, GenerateHardness4EveryPoint)
{
	CInitialClusterCreator Creator;
	Creator.testGenerateHardness4EveryPoint()
}

TEST(Test_CreateInitialCluster, ComputeClusterCenter)
{
}

TEST(Test_CreateInitialCluster, DivideUserSpecifiedRegion)
{
}

TEST(Test_CreateInitialCluster, CreateInitialCluster)
{
}