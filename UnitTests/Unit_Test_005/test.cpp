#include "pch.h"
#include "gtest/gtest.h"
#include "PointCloudAutoRetouchScene.h"
#include "PointCloudVisualizer.h"
//#include "AutoRetouchInterface.h"
//#include "VisualizationInterface.h"
#include "pcl/io/pcd_io.h"

using namespace hiveObliquePhotography::Visualization;

TEST(Test_PointCloudVisualizer, TestInitAndRefresh)
{
	pcl::PointCloud<pcl::PointSurfel>* pCloud = new pcl::PointCloud<pcl::PointSurfel>;
	pcl::io::loadPCDFile("Panda.pcd", *pCloud);

	hiveObliquePhotography::AutoRetouch::CPointCloudAutoRetouchScene::getInstance()->init(pCloud);
	CPointCloudVisualizer::getInstance()->init(pCloud);
	CPointCloudVisualizer::getInstance()->refresh();

	//hiveObliquePhotography::AutoRetouch::hiveInitPointCloudScene(pCloud);
	//hiveObliquePhotography::Visualization::hiveInitVisualizer(pCloud);
	//hiveObliquePhotography::Visualization::hiveRefreshVisualizer();

	system("pause");
}