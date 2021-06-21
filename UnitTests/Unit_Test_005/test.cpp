#include "pch.h"
#include "gtest/gtest.h"
#include "ObliquePhotographyDataInterface.h"
#include "PointCloudRetouchInterface.h"
#include "VisualizationInterface.h"
#include "PointCloudRetouchConfig.h"
#include "pcl/io/pcd_io.h"

using namespace hiveObliquePhotography;

TEST(Test_PointCloudVisualizer, TestInitAndRefresh)
{
	const std::vector<std::string> FilePaths
	{
		//"TestModel/Tile_1_L19.pcd",
		//"TestModel/Tile_1_L19_Down_normal.pcd",
		//"TestModel/slice 1.pcd", 
		//"TestModel/slice 2.pcd", 
		"TestModel/slice 3.pcd", 
		//"TestModel/slice 4.pcd"  
	};
	//
	//auto pCloud = hiveObliquePhotography::CPointCloudScene::getInstance()->loadScene(FilePaths);

	//hiveObliquePhotography::AutoRetouch::CPointCloudAutoRetouchScene::getInstance()->init(pCloud);
	//CPointCloudVisualizer::getInstance()->init(pCloud);
	//CPointCloudVisualizer::getInstance()->refresh();
	//
	//system("pause");

	hiveConfig::CHiveConfig* pConfig = new PointCloudRetouch::CPointCloudRetouchConfig;
	hiveConfig::hiveParseConfig("PointCloudRetouchConfig.xml", hiveConfig::EConfigType::XML, pConfig);

	PointCloud_t::Ptr pCloud(new PointCloud_t);
	pcl::io::loadPCDFile(FilePaths.front(), *pCloud);

	//auto pCloud = hiveInitPointCloudScene(FilePaths);
	PointCloudRetouch::hiveInit(pCloud, pConfig);
	Visualization::hiveInitVisualizer(pCloud, false);
	std::vector<std::size_t> PointLabel;
	PointCloudRetouch::hiveDumpPointLabel4Visualizer(PointLabel);
	Visualization::hiveRefreshVisualizer(PointLabel, true);
	Visualization::hiveRunVisualizerLoop();
}