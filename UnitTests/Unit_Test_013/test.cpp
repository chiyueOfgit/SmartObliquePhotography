#include "pch.h"
#include "PointCloudRetouchInterface.h"
#include "PointLabelSet.h"

//	测试用例列表：
//	* Uninitialized_Test：测试未初始化时能否正确运行；
//	* Illegal_Input_Test：测试输入非法时能否正确运行；
//	* 由于CPointLabelSet为数据类，所以没必要进行功能上的测试

using namespace hiveObliquePhotography::PointCloudRetouch;

void tryCatchAndThrow(std::function<void()> vDoSomething)
{
	try
	{
		vDoSomething();
	}
	catch (const std::exception& e)
	{
		std::cout << e.what() << std::endl;
		throw e;
	}
	catch (...)
	{
		std::cout << "unknown exception" << std::endl;
		throw std::exception("unknown exception");
	}
}

TEST(TestPointLabelSet, Uninitialized_Test)
{
	auto DoSomething = []()
	{
		CPointLabelSet PointLabelSet;

		PointLabelSet.tagPointLabel(999, {}, 999, {});
		PointLabelSet.tagCoreRegion4Cluster({ 999 }, {}, 999);
		PointLabelSet.getLabelAt(999);
		PointLabelSet.getClusterIndexAt(999);
		PointLabelSet.getClusterBelongingProbabilityAt(999);
	};

	ASSERT_NO_THROW(DoSomething());
}

TEST(TestPointLabelSet, Illegal_Input_Test)
{
	auto DoSomething = []()
	{
		CPointLabelSet PointLabelSet;
		PointLabelSet.init(-1);
		PointLabelSet.init(100);

		PointLabelSet.tagPointLabel(-1, {}, -1, {});
		PointLabelSet.tagPointLabel(999, {}, 999, {});

		PointLabelSet.tagCoreRegion4Cluster({}, {}, 50);
		PointLabelSet.tagCoreRegion4Cluster({ -1 }, {}, -1);
		PointLabelSet.tagCoreRegion4Cluster({ 999 }, {}, 999);

		PointLabelSet.getLabelAt(-1);
		PointLabelSet.getLabelAt(999);

		PointLabelSet.getClusterIndexAt(-1);
		PointLabelSet.getClusterIndexAt(999);

		PointLabelSet.getClusterBelongingProbabilityAt(-1);
		PointLabelSet.getClusterBelongingProbabilityAt(999);
	};
	
	ASSERT_NO_THROW(DoSomething());
}
