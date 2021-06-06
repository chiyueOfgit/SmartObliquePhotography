#include "pch.h"
#include "AutoRetouchInterface.h"
#include "PointCloudAutoRetouchScene.h"
#include "PointCloudVisualizer.h"
#include "pcl/io/pcd_io.h"
#include "PointCluster4VFH.h"

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
const std::string g_KeptGroundPoints = "SomeGroundPoints.txt";

TEST(Test_Binary, BinaryByVFH)
{
	pcl::PointCloud<pcl::PointSurfel>* pCloud = new pcl::PointCloud<pcl::PointSurfel>;
	pcl::io::loadPCDFile(g_Folder + g_CloudFile, *pCloud);

	hiveObliquePhotography::AutoRetouch::CPointCloudAutoRetouchScene::getInstance()->init(pCloud);

	std::vector<IPointCluster*> pClusters;
	pClusters.push_back(new CPointCluster4VFH(loadPointIndices(g_Folder + g_UnwantedTreePoints), EPointLabel::UNWANTED));
	pClusters.push_back(new CPointCluster4VFH(loadPointIndices(g_Folder + g_KeptGroundPoints), EPointLabel::KEPT));

	hiveObliquePhotography::AutoRetouch::hiveExecuteBinaryClassifier(hiveObliquePhotography::AutoRetouch::CLASSIFIER_BINARY_VFH, pClusters);

	hiveObliquePhotography::Visualization::CPointCloudVisualizer::getInstance()->init(pCloud);
	hiveObliquePhotography::Visualization::CPointCloudVisualizer::getInstance()->refresh();

	system("pause");
}