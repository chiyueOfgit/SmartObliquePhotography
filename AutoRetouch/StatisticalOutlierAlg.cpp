#include "pch.h"
#include "StatisticalOutlierAlg.h"
#include "PointCloudAutoRetouchScene.h"
#include "PointLabel4Classfier.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>


using namespace hiveObliquePhotography::AutoRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CStaOutlierDetectingAlg, CLASSIFIER_OUTLIER_DETECTION)


//*****************************************************************
//FUNCTION:
void  CStaOutlierDetectingAlg::runV(std::vector<std::uint64_t>& vioInputSet, EPointLabel vFinalLabel)
{
	const auto& pScene = CPointCloudAutoRetouchScene::getInstance();
	const auto& pCloud = pScene->getPointCloudScene();
	auto& pKdTree = pScene->getGlobalKdTree();

	pcl::PointCloud<pcl::PointSurfel>::Ptr pTempCloud(new pcl::PointCloud<pcl::PointSurfel>);
	pcl::PointCloud<pcl::PointSurfel>::Ptr pResultCloud(new pcl::PointCloud<pcl::PointSurfel>);

	for (auto Index : vioInputSet)
	{
		pcl::PointSurfel Point = pCloud->points[Index];
		Point.curvature = Index;
		pTempCloud->push_back(Point);
	}

	pcl::StatisticalOutlierRemoval<pcl::PointSurfel> Od;
	Od.setInputCloud(pTempCloud);
	Od.setMeanK(50);
	Od.setStddevMulThresh(0.50);
	Od.filter(*pResultCloud);

	std::vector<std::uint64_t> innerIndex;
	for (auto& Point : pResultCloud->points)
	{
		innerIndex.push_back(Point.curvature);
	}
	vioInputSet.swap(innerIndex);

	Od.setNegative(true);
	Od.filter(*pResultCloud);

	std::vector<SPointLabelChange> PointLabelChangeRecord;
	for (auto& Point : pResultCloud->points)
	{
		m_pLocalLabelSet->changePointLabel(Point.curvature, vFinalLabel);
	}

}