#include "pch.h"
#include "PointCloudRetouchInterface.h"
#include "PointCloudRetouchScene.h"
#include "PointLabelSet.h"
#include "NeighborhoodBuilder.h"
#include "RetouchTask.h"
#include "PointCloudRetouchManager.h"
#include "PointCloudRetouchConfig.h"
#include <pcl/point_types.h>
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/visualization/common/common.h"

#include <fstream>
#include <boost/archive/text_oarchive.hpp> 
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

using namespace hiveObliquePhotography::PointCloudRetouch;

//测试用例列表：
//  * LabelSet_Undo_Overview_Test:   是否成功撤销对LabelSet的更改
//  * Timestamp_Undo_Overview_Test:  是否成功撤销对Timestamp的更改
//
//  * LabelSet_Undo_Cleanup_Test:    对LabelSet的撤销不应该影响下次执行的结果
//  * Timestamp_Undo_Cleanup_Test:   对Timestamp的撤销不应该影响下次执行的结果
// 
//  * Empty_ResultQueue_Expect_Test: 对空的结果队列进行撤销不应引起异常
//  * Empty_Input_Expect_Test:       用户空选时，当前状态不应该加入结果队列

using namespace  hiveObliquePhotography::PointCloudRetouch;

const std::string ConfigPath = TESTMODEL_DIR + std::string("Config/Test008_PointCloudRetouchConfig.xml");
const std::string ModelPath = TESTMODEL_DIR + std::string("General/slice 16.pcd");
const std::string IndicesPath = TESTMODEL_DIR + std::string("Test008_Model/CompleteBuildingInput.txt");
const std::string CameraPath = TESTMODEL_DIR + std::string("Test008_Model/CompleteBuildingCameraInfo.txt");

class CTestUndo : public testing::Test
{
public:
	hiveConfig::CHiveConfig* pConfig = nullptr;
	CPointCloudRetouchManager* pManager = nullptr;
protected:
	void SetUp() override
	{
		pConfig = new CPointCloudRetouchConfig;
		if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, pConfig) != hiveConfig::EParseResult::SUCCEED)
		{
			_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
			return;
		}
	}

	void initTest(const std::string& vModelPath)
	{
		PointCloud_t::Ptr pCloud(new PointCloud_t);
		pcl::io::loadPCDFile(vModelPath, *pCloud);
		ASSERT_GT(pCloud->size(), 0);
		pManager = CPointCloudRetouchManager::getInstance();
		pManager->init(pCloud, pConfig);
	}

	void loadIndices(const std::string& vPath, pcl::Indices& voIndices)
	{
		std::ifstream File(vPath);
		boost::archive::text_iarchive ia(File);
		ia >> BOOST_SERIALIZATION_NVP(voIndices);
		File.close();
	}

	void expandOnce(const std::string& vIndicesPath, const std::string& vCameraPath)
	{
		pcl::Indices Indices;
		if (!vIndicesPath.empty())
			loadIndices(vIndicesPath, Indices);

		pcl::visualization::Camera Camera;
		auto pVisualizer = new pcl::visualization::PCLVisualizer("Viewer", true);
		pVisualizer->loadCameraParameters(vCameraPath);
		pVisualizer->getCameraParameters(Camera);
		Eigen::Matrix4d ViewMatrix, ProjectionMatrix;
		Camera.computeViewMatrix(ViewMatrix);
		Camera.computeProjectionMatrix(ProjectionMatrix);
		
		hiveMarkLitter(Indices, 0.8, 10, { 200, 300 }, ProjectionMatrix * ViewMatrix, { 1000, 1000 });
	}
	
	void TearDown() override
	{
		delete pConfig;
	}
};

TEST_F(CTestUndo, Empty_ResultQueue_Expect_Test)
{
	initTest(ModelPath);

	EXPECT_FALSE(pManager->undo());
	ASSERT_NO_FATAL_FAILURE(pManager->undo());
	ASSERT_NO_THROW(pManager->undo());
}

TEST_F(CTestUndo, LabelSet_Undo_Overview_Test)
{
	initTest(ModelPath);
	std::vector<std::size_t> LabelSetBeforeUndo, LabelSetAfterUndo;
	
	hiveDumpPointLabel(LabelSetBeforeUndo);
	expandOnce(IndicesPath, CameraPath);
	hiveUndo();
	hiveDumpPointLabel(LabelSetAfterUndo);

	std::vector<std::size_t> SymmetricDifference;
	std::set_symmetric_difference(LabelSetBeforeUndo.begin(), LabelSetBeforeUndo.end(),
		LabelSetAfterUndo.begin(), LabelSetAfterUndo.end(),
		std::inserter(SymmetricDifference, SymmetricDifference.begin()));
	ASSERT_EQ(SymmetricDifference.size(), 0);
}

TEST_F(CTestUndo, Timestamp_Undo_Overview_Test)
{
	initTest(ModelPath);
	
	const auto TimestampBeforeUndo = pManager->addAndGetTimestamp();
	expandOnce(IndicesPath, CameraPath);
	hiveUndo();
	const auto TimestampAfterUndo = pManager->addAndGetTimestamp();

	ASSERT_EQ(TimestampBeforeUndo, TimestampAfterUndo);
}

TEST_F(CTestUndo, LabelSet_Undo_Cleanup_Test)
{
	initTest(ModelPath);
	std::vector<std::size_t> LabelSetBeforeUndo, LabelSetAfterUndo;

	expandOnce(IndicesPath, CameraPath);
	hiveDumpPointLabel(LabelSetBeforeUndo);
	hiveUndo();
	expandOnce(IndicesPath, CameraPath);
	hiveDumpPointLabel(LabelSetAfterUndo);
	
	std::vector<std::size_t> SymmetricDifference;
	std::set_symmetric_difference(LabelSetBeforeUndo.begin(), LabelSetBeforeUndo.end(),
		LabelSetAfterUndo.begin(), LabelSetAfterUndo.end(),
		std::inserter(SymmetricDifference, SymmetricDifference.begin()));
	ASSERT_EQ(SymmetricDifference.size(), 0);
}

TEST_F(CTestUndo, Timestamp_Undo_Cleanup_Test)
{
	initTest(ModelPath);

	expandOnce(IndicesPath, CameraPath);
	const auto TimestampBeforeUndo = pManager->addAndGetTimestamp();
	hiveUndo();
	expandOnce(IndicesPath, CameraPath);
	const auto TimestampAfterUndo = pManager->addAndGetTimestamp();

	ASSERT_EQ(TimestampBeforeUndo, TimestampAfterUndo);
}

TEST_F(CTestUndo, Empty_Input_Expect_Test)
{
	initTest(ModelPath);

	expandOnce({}, CameraPath);

	EXPECT_FALSE(pManager->undo());
	ASSERT_NO_FATAL_FAILURE(pManager->undo());
	ASSERT_NO_THROW(pManager->undo());
}