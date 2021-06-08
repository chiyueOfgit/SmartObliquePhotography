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
	std::string m_Paths[3] =
	{
		//较近的聚类可见面积最大
		"testcase/LeftBigTree.indices.txt\n"
		"testcase/LeftBigTree.camera.txt\n"
		"groundtruth/LeftBigTree.txt\n",

		//较远的聚类可见面积最大
		"testcase/Road.indices.txt\n"
		"testcase/Road.camera.txt\n"
		"groundtruth/Road.txt\n",

		//不可见的聚类面积最大
		"testcase/RightFourTrees.indices.txt\n"
		"testcase/RightFourTrees.camera.txt\n"
		"groundtruth/RightFourTrees.txt\n",
	};
	std::string m_Sinple_Paths[3] =
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
		hiveObliquePhotography::AutoRetouch::CPointCloudAutoRetouchScene::getInstance()->init(pCloud);
	}

	void TearDown() override
	{
	}

	std::tuple<std::vector<size_t>, pcl::visualization::Camera, std::vector<size_t>> _loadTestcase(const std::string_view& vPaths) const
	{
		std::vector<std::string_view> Path;
		__lines(vPaths, Path);

		for (const auto StringView : Path)
			std::cout << StringView << std::endl;

		std::vector<size_t> InputSet;
		pcl::visualization::Camera Camera;
		std::vector<size_t> GroundTruth;

		__loadIndices(Path[0], InputSet);
		auto pVisualizer = new pcl::visualization::PCLVisualizer("Viewer",true);
		pVisualizer->loadCameraParameters(Path[1].data());
		pVisualizer->getCameraParameters(Camera);
		__loadIndices(Path[2], GroundTruth);

		return { InputSet, Camera, GroundTruth };
	}

private:
	static void __lines(const std::string_view& vStr, std::vector<std::string_view>& voStrs)
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

	static void __loadIndices(const std::string_view& vPath, std::vector<size_t>& voIndices)
	{
		const std::string Path{ vPath };
		std::ifstream File(Path);
		boost::archive::text_iarchive ia(File);
		ia >> BOOST_SERIALIZATION_NVP(voIndices);
		File.close();
	}
};


TEST_F(TestAreaPicking, 实用测试1，较近的聚类可见面积最大)
{
	const auto& Path = m_Sinple_Paths[0];

	auto [InputSet, Camera, GroundTruth] = _loadTestcase(Path);

	hiveObliquePhotography::AutoRetouch::hiveExecuteClusteringClassifier(hiveObliquePhotography::AutoRetouch::CLASSIFIER_MaxVisibilityCluster, InputSet, hiveObliquePhotography::AutoRetouch::EPointLabel::KEPT,Camera);

	std::vector<size_t> Difference;
	std::set_difference(InputSet.begin(), InputSet.end(),
		GroundTruth.begin(), GroundTruth.end(),
		std::inserter(Difference, Difference.begin()));

	GTEST_ASSERT_EQ(Difference.size(), 0u);
}

TEST_F(TestAreaPicking, 实用测试2，较远的聚类可见面积最大)
{
	const auto& Path = m_Sinple_Paths[1];

	auto [InputSet, Camera, GroundTruth] = _loadTestcase(Path);

	hiveObliquePhotography::AutoRetouch::hiveExecuteClusteringClassifier(hiveObliquePhotography::AutoRetouch::CLASSIFIER_MaxVisibilityCluster, InputSet, hiveObliquePhotography::AutoRetouch::EPointLabel::KEPT, Camera);

	std::vector<size_t> Difference;
	std::set_difference(InputSet.begin(), InputSet.end(),
		GroundTruth.begin(), GroundTruth.end(),
		std::inserter(Difference, Difference.begin()));

	GTEST_ASSERT_EQ(Difference.size(), 0u);

}

TEST_F(TestAreaPicking, 实用测试3，不可见的聚类面积最大)
{
	const auto& Path = m_Sinple_Paths[2];

	auto [InputSet, Camera, GroundTruth] = _loadTestcase(Path);

	hiveObliquePhotography::AutoRetouch::hiveExecuteClusteringClassifier(hiveObliquePhotography::AutoRetouch::CLASSIFIER_MaxVisibilityCluster, InputSet, hiveObliquePhotography::AutoRetouch::EPointLabel::KEPT, Camera);

	std::vector<size_t> Difference;
	std::set_difference(InputSet.begin(), InputSet.end(),
		GroundTruth.begin(), GroundTruth.end(),
		std::inserter(Difference, Difference.begin()));

	GTEST_ASSERT_EQ(Difference.size(), 0u);
}