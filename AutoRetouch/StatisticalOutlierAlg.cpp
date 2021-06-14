#include "pch.h"
#include "StatisticalOutlierAlg.h"
#include "PointCloudAutoRetouchScene.h"
#include "PointLabel4Classfier.h"
#include <pcl/filters/statistical_outlier_removal.h>

using namespace hiveObliquePhotography::AutoRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CStaOutlierDetectingAlg, CLASSIFIER_OUTLIER_DETECTION)


//*****************************************************************
//FUNCTION:
void  CStaOutlierDetectingAlg::runV(pcl::Indices& vioInputSet, EPointLabel vExpectLabel)
{
	if (hiveConfig::hiveParseConfig("AutoRetouchConfig.xml", hiveConfig::EConfigType::XML, CAutoRetouchConfig::getInstance()) != hiveConfig::EParseResult::SUCCEED)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", "AutoRetouchConfig.xml"));
		return;
	}

	const auto& pCloud = CPointCloudAutoRetouchScene::getInstance()->getPointCloudScene();
	auto& pKdTree = CPointCloudAutoRetouchScene::getInstance()->getGlobalKdTree();

	pcl::Indices OutlierIndices;
	pcl::StatisticalOutlierRemoval<pcl::PointSurfel> Od;
	Od.setInputCloud(pCloud);
	Od.setIndices(pcl::make_shared<pcl::Indices>(vioInputSet.begin(), vioInputSet.end()));
	Od.setMeanK(*CAutoRetouchConfig::getInstance()->getAttribute<int>("OUTLIER_MEAN_KNN_NUMBER"));
	Od.setStddevMulThresh(*CAutoRetouchConfig::getInstance()->getAttribute<float>("OUTLIER_STD_MULTIPLE_THRESHOLD"));
	Od.setNegative(true);
	Od.filter(OutlierIndices);

	for (auto& Index : OutlierIndices)
	{
		m_pLocalLabelSet->changePointLabel(Index, vExpectLabel);
	}

}