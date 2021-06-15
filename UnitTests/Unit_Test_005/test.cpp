#include "pch.h"
#include "gtest/gtest.h"
#include "PointCloudScene.h"
#include "PointCloudAutoRetouchScene.h"
#include "PointCloudVisualizer.h"
#include "ObliquePhotographyDataInterface.h"
#include "AutoRetouchInterface.h"
#include "VisualizationInterface.h"

using namespace hiveObliquePhotography;

TEST(Test_PointCloudVisualizer, TestInitAndRefresh)
{
	const std::vector<std::string> FilePaths
	{
		"TestModel/Tile_1_L19.pcd",
		//"TestModel/Tile_1_L19_Down_normal.pcd",
		//"TestModel/slice 1.pcd", 
		//"TestModel/slice 2.pcd", 
		//"TestModel/slice 3.pcd", 
		//"TestModel/slice 4.pcd"  
	};
	//
	//auto pCloud = hiveObliquePhotography::CPointCloudScene::getInstance()->loadScene(FilePaths);

	//hiveObliquePhotography::AutoRetouch::CPointCloudAutoRetouchScene::getInstance()->init(pCloud);
	//CPointCloudVisualizer::getInstance()->init(pCloud);
	//CPointCloudVisualizer::getInstance()->refresh();
	//
	//system("pause");

	auto pCloud = hiveInitPointCloudScene(FilePaths);
	AutoRetouch::hiveInitPointCloudScene(pCloud);
	Visualization::hiveInitVisualizer(pCloud, false);
	Visualization::hiveRefreshVisualizer(true);
	Visualization::hiveRunVisualizerLoop();
}