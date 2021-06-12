#include "pch.h"

#include "AutoRetouchInterface.h"
#include "PointCloudAutoRetouchScene.h"

#include <fstream>
#include <boost/archive/text_oarchive.hpp> 
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <pcl/io/pcd_io.h>

using namespace hiveObliquePhotography::AutoRetouch;

class TestAreaPicking : public testing::Test
{
protected:
	std::string m_SamplePaths[3] =
	{
		//较近的聚类可见面积最大
		"testcase/SimpleTestOne.indices.txt\n"
		"testcase/SimpleTestOne.camera.txt\n"
		"groundtruth/LeftBigTree.txt\n",

		//较远的聚类可见面积最大
		"testcase/SimpleTestTwo.indices.txt\n"
		"testcase/SimpleTestTwo.camera.txt\n"
		"groundtruth/Road.txt\n",

		//不可见的聚类面积最大
		"testcase/SimpleTestThree.indices.txt\n"
		"testcase/SimpleTestThree.camera.txt\n"
		"groundtruth/RightFourTrees.txt\n",
	};

	void SetUp() override
	{
		std::string ModelPath("test_tile16/Scu_Tile16.pcd");
		pcl::PointCloud<pcl::PointSurfel>::Ptr pCloud(new pcl::PointCloud<pcl::PointSurfel>);
		pcl::io::loadPCDFile(ModelPath, *pCloud);
		CPointCloudAutoRetouchScene::getInstance()->init(pCloud);
	}

	void TearDown() override
	{
	}

	static std::tuple<pcl::IndicesPtr, pcl::visualization::Camera, pcl::IndicesPtr> _loadTestcase(const std::string_view& vPaths);

private:
	static void __lines(const std::string_view& vStr, std::vector<std::string_view>& voStrs);
	static void __loadIndices(const std::string_view& vPath, pcl::Indices& voIndices);
};


TEST_F(TestAreaPicking, NearestClusterWithMaxVisibility)
{
	const auto& Path = m_SamplePaths[0];

	auto [pTestee, Camera, pGroundTruth] = _loadTestcase(Path);

	hiveExecuteMaxVisibilityClustering(pTestee, EPointLabel::KEPT,Camera);
	std::vector<size_t> Difference;
	std::set_difference(pTestee->begin(), pTestee->end(),
		pGroundTruth, pGroundTruth,
		std::inserter(Difference, Difference.begin()));

	GTEST_ASSERT_EQ(Difference.size(), 0u);
}

TEST_F(TestAreaPicking, FurthestClusterWithMaxVisibility)
{
	const auto& Path = m_SamplePaths[1];

	auto [pTestee, Camera, pGroundTruth] = _loadTestcase(Path);

	hiveExecuteMaxVisibilityClustering(pTestee, EPointLabel::KEPT, Camera);
	std::vector<size_t> Difference;
	std::set_difference(pTestee->begin(), pTestee->end(),
		pGroundTruth, pGroundTruth,
		std::inserter(Difference, Difference.begin()));

	GTEST_ASSERT_EQ(Difference.size(), 0u);
}

TEST_F(TestAreaPicking, InvisibleClusterWithMaxArea)
{
	const auto& Path = m_SamplePaths[2];

	auto [pTestee, Camera, pGroundTruth] = _loadTestcase(Path);

	hiveExecuteMaxVisibilityClustering(pTestee, EPointLabel::KEPT, Camera);
	std::vector<size_t> Difference;
	std::set_difference(pTestee->begin(), pTestee->end(),
		pGroundTruth, pGroundTruth,
		std::inserter(Difference, Difference.begin()));

	GTEST_ASSERT_EQ(Difference.size(), 0u);
}

TEST_F(TestAreaPicking, DeathTest_IndicesIsNullptr)
{
	pcl::IndicesPtr pTestee = nullptr;
	pcl::visualization::Camera Camera;
	EXPECT_DEATH(hiveExecuteMaxVisibilityClustering(pTestee, EPointLabel::KEPT, Camera); , "");
}

//*****************************************************************
//FUNCTION: 
std::tuple<pcl::IndicesPtr, pcl::visualization::Camera, pcl::IndicesPtr> TestAreaPicking::_loadTestcase(const std::string_view& vPaths)
{
	std::vector<std::string_view> Path;
	__lines(vPaths, Path);

	for (const auto StringView : Path)
		std::cout << StringView << std::endl;

	pcl::IndicesPtr pTestee(new pcl::Indices);
	pcl::visualization::Camera Camera;
	pcl::IndicesPtr pGroundTruth(new pcl::Indices);

	__loadIndices(Path[0], *pTestee);
	auto pVisualizer = new pcl::visualization::PCLVisualizer("Viewer", true);
	pVisualizer->loadCameraParameters(Path[1].data());
	pVisualizer->getCameraParameters(Camera);
	__loadIndices(Path[2], *pGroundTruth);

	return { pTestee, Camera, pGroundTruth };
}

//*****************************************************************
//FUNCTION: 
void TestAreaPicking::__lines(const std::string_view& vStr, std::vector<std::string_view>& voStrs)
{
	const auto Sep = "\n";
	size_t Start = vStr.find_first_not_of(Sep);
	while (Start != std::string_view::npos)
	{
		size_t End = vStr.find_first_of(Sep, Start + 1);
		if (End == std::string_view::npos)
			End = vStr.length();

		voStrs.push_back(vStr.substr(Start, End - Start));
		Start = vStr.find_first_not_of(Sep, End + 1);
	}
}

//*****************************************************************
//FUNCTION: 
void TestAreaPicking::__loadIndices(const std::string_view& vPath, pcl::Indices& voIndices)
{
	const std::string Path{ vPath };
	std::ifstream File(Path);
	boost::archive::text_iarchive ia(File);
	ia >> BOOST_SERIALIZATION_NVP(voIndices);
	File.close();
}