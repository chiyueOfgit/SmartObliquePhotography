#include "pch.h"
#include "gtest/gtest.h"
#include "PointCloudVisualizer.h"

using namespace hiveObliquePhotography::visualizer;

TEST(Test_PointCloudVisualizer, TestName)
{
	CPointCloudVisualizer::getInstance()->refresh();
	system("pause");
}