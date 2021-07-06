#include "pch.h"

#include <common/MathInterface.h>
#include "PointCloudRetouchInterface.h"
#include "PointCloudRetouchConfig.h"
#include <fstream>
#include <boost/archive/text_oarchive.hpp> 
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <pcl/io/pcd_io.h>

#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/visualization/common/common.h"

//测试用例列表：
//SelectingBaseTest:给出明显的判别逻辑情况，证明选择的正确性；
//  * Selecting_NoThrough_Test:近距离不能选透；
//  * Selecting_MoreTree_Test:可以选择多棵树；
//  * Selecting_MinCulling_Test_1:选择多棵树避开房子；
//  * Selecting_MinCulling_Test_2:选择地面避开树；
//SelectingSpecialTest特定情况下的特殊结果正确

std::string SamplePaths[4][3] =
{
	{
		"testcase/SimpleTestOne.indices.txt",
		"testcase/SimpleTestOne.camera.txt",
		"groundtruth/LeftBigTree.txt",
	},

	{
		"testcase/SimpleTestOne.indices.txt",
		"testcase/SimpleTestOne.camera.txt",
		"groundtruth/LeftBigTree.txt",
	},

	{
		"testcase/SimpleTestOne.indices.txt",
		"testcase/SimpleTestOne.camera.txt",
		"groundtruth/LeftBigTree.txt",
	},

	{
		"testcase/SimpleTestOne.indices.txt",
		"testcase/SimpleTestOne.camera.txt",
		"groundtruth/LeftBigTree.txt",
	}
	
};


using namespace hiveObliquePhotography::PointCloudRetouch;

class TestSelecting : public testing::Test
{
protected:
	void SetUp() override
	{
		std::string ModelPath("test_tile16/Scu_Tile16.pcd");
		PointCloud_t::Ptr pCloud(new PointCloud_t);
		pcl::io::loadPCDFile(ModelPath, *pCloud);
		
	}

	void TearDown() override
	{
	}

	static void _loadIndices(const std::string& vPath, pcl::Indices& voIndices);
	static void _loadCamera(const std::string& vPath, pcl::visualization::Camera& voCamera);
};

//*****************************************************************
//FUNCTION: 
void TestSelecting::_loadIndices(const std::string& vPath, pcl::Indices& voIndices)
{
	std::ifstream File(vPath);
	boost::archive::text_iarchive ia(File);
	ia >> BOOST_SERIALIZATION_NVP(voIndices);
	File.close();
}

//*****************************************************************
//FUNCTION: 
void TestSelecting::_loadCamera(const std::string& vPath, pcl::visualization::Camera& voCamera)
{
	auto pVisualizer = new pcl::visualization::PCLVisualizer("Viewer", true);
	pVisualizer->loadCameraParameters(vPath);
	pVisualizer->getCameraParameters(voCamera);
}