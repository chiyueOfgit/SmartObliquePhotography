#include "pch.h"
#include "PointCloudRetouchInterface.h"
#include "PointLabelSet.h"

//	测试用例列表：
//	* Uninitialized_Test：测试未初始化时能否正确报错，报错后能否继续正确运行
//	* Illegal_Input_Test：测试输入非法时能否正确报错，报错后能否继续正确运行
//	* 由于CPointLabelSet为数据类，所以没必要进行功能上的测试

using namespace hiveObliquePhotography::PointCloudRetouch;

TEST(TestPointLabelSet, Uninitialized_Test)
{
	CPointLabelSet PointLabelSet;

	EXPECT_ANY_THROW(PointLabelSet.tagPointLabel(999, {}, 999, {}));
	EXPECT_ANY_THROW(PointLabelSet.tagCoreRegion4Cluster({ 999 }, {}, 999));
	EXPECT_ANY_THROW(PointLabelSet.getLabelAt(999));
	EXPECT_ANY_THROW(PointLabelSet.getClusterIndexAt(999));
	EXPECT_ANY_THROW(PointLabelSet.getClusterBelongingProbabilityAt(999));

	EXPECT_NO_THROW(PointLabelSet.init(100));
	EXPECT_NO_THROW(PointLabelSet.tagPointLabel(50, {}, 50, 0.5));
	EXPECT_NO_THROW(PointLabelSet.tagCoreRegion4Cluster({ 50 }, {}, 50));
	EXPECT_NO_THROW(PointLabelSet.getLabelAt(50));
	EXPECT_NO_THROW(PointLabelSet.getClusterIndexAt(50));
	EXPECT_NO_THROW(PointLabelSet.getClusterBelongingProbabilityAt(50));
}

TEST(TestPointLabelSet, Illegal_Input_Test)
{
	CPointLabelSet PointLabelSet;
	EXPECT_ANY_THROW(PointLabelSet.init(-1));
	EXPECT_NO_THROW(PointLabelSet.init(100));

	EXPECT_ANY_THROW(PointLabelSet.tagPointLabel(-1, {}, -1, 0.5));
	EXPECT_ANY_THROW(PointLabelSet.tagPointLabel(999, {}, 999, 0.5));
	EXPECT_ANY_THROW(PointLabelSet.tagPointLabel(50, {}, 50, -1));
	EXPECT_ANY_THROW(PointLabelSet.tagPointLabel(50, {}, 50, 10));
	EXPECT_ANY_THROW(PointLabelSet.tagCoreRegion4Cluster({}, {}, 50));
	EXPECT_ANY_THROW(PointLabelSet.tagCoreRegion4Cluster({ -1 }, {}, -1));
	EXPECT_ANY_THROW(PointLabelSet.tagCoreRegion4Cluster({ 999 }, {}, 999));
	EXPECT_ANY_THROW(PointLabelSet.getLabelAt(-1));
	EXPECT_ANY_THROW(PointLabelSet.getLabelAt(999));
	EXPECT_ANY_THROW(PointLabelSet.getClusterIndexAt(-1));
	EXPECT_ANY_THROW(PointLabelSet.getClusterIndexAt(999));
	EXPECT_ANY_THROW(PointLabelSet.getClusterBelongingProbabilityAt(-1));
	EXPECT_ANY_THROW(PointLabelSet.getClusterBelongingProbabilityAt(999));

	EXPECT_NO_THROW(PointLabelSet.tagPointLabel(50, {}, 50, 0.5));
	EXPECT_NO_THROW(PointLabelSet.tagCoreRegion4Cluster({ 50 }, {}, 50));
	EXPECT_NO_THROW(PointLabelSet.getLabelAt(50));
	EXPECT_NO_THROW(PointLabelSet.getClusterIndexAt(50));
	EXPECT_NO_THROW(PointLabelSet.getClusterBelongingProbabilityAt(50));
}
