#include "pch.h"
#include "gtest/gtest.h"
#include "PointCloudVisualizer.h"
#include "pcl/io/pcd_io.h"

using namespace hiveObliquePhotography::visualizer;

TEST(Test_PointCloudVisualizer, TestName)
{
	pcl::PointCloud<pcl::PointSurfel>::Ptr pCloud(new pcl::PointCloud<pcl::PointSurfel>);
	pcl::io::loadPCDFile("Panda.pcd", *pCloud);

	CPointCloudVisualizer::getInstance()->init(pCloud);

	CPointCloudVisualizer::getInstance()->refresh();
	system("pause");
}