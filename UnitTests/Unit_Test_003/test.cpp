#include "pch.h"
#include "AutoRetouchInterface.h"
#include "PointCloudAutoRetouchScene.h"
#include "PointCloudVisualizer.h"
#include <pcl/io/pcd_io.h>

#include <fstream>
#include <boost/archive/text_oarchive.hpp> 
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>


using namespace hiveObliquePhotography::AutoRetouch;

std::vector<std::size_t> loadPointIndices(std::string vPath)
{
	std::vector<std::size_t> Indices;
	std::ifstream File(vPath.c_str());
	boost::archive::text_iarchive ia(File);
	ia >> BOOST_SERIALIZATION_NVP(Indices);
	File.close();	//ia后才能关闭
	return Indices;
}

const std::string g_Folder = "test_tile16/";
const std::string g_CloudFile = "Scu_Tile16.pcd";
const std::string g_UnwantedTreePoints = "SomeBigTreePoints.txt";

TEST(Test_RegionGrowing, RegionGrowingByColor)
{
	pcl::PointCloud<pcl::PointSurfel>* pCloud = new pcl::PointCloud<pcl::PointSurfel>;
	pcl::io::loadPCDFile(g_Folder + g_CloudFile, *pCloud);

	hiveObliquePhotography::AutoRetouch::CPointCloudAutoRetouchScene::getInstance()->init(pCloud);

	std::vector<std::uint64_t> pClusters = { 18,69};

	hiveObliquePhotography::AutoRetouch::hiveExecuteRegionGrowClassifier(hiveObliquePhotography::AutoRetouch::CLASSIFIER_REGION_GROW_COLOR, pClusters, EPointLabel::UNWANTED);

	hiveObliquePhotography::Visualization::CPointCloudVisualizer::getInstance()->init(pCloud);
	hiveObliquePhotography::Visualization::CPointCloudVisualizer::getInstance()->refresh();

	system("pause");
}






