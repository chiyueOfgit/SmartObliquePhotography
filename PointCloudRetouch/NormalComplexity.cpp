#include "pch.h"
#include "NormalComplexity.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/features/don.h"
#include "EuclideanNeighborhoodBuilder.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CNormalComplexity, KEYWORD::NORMAL_COMPLEXITY)

//*****************************************************************
//FUNCTION: 
double CNormalComplexity::generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter)
{
	const auto& CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();
	
	if (vDeterminantPointSet.empty() || vValidationSet.empty())
		return 0.0;

	m_AverageDon = __calcPointCloudNormalComplexity(vDeterminantPointSet);
	auto ValidationFeature = __calcPointCloudNormalComplexity(vValidationSet);
	return 1.0 - static_cast<double>(abs(ValidationFeature - m_AverageDon));
}

//*****************************************************************
//FUNCTION: 
double CNormalComplexity::evaluateFeatureMatchFactorV(pcl::index_t vInputPoint)
{
	const double LargeScaleRadius = *m_pConfig->getAttribute<double>("LARGE_SCALE_RADIUS");
	
	std::vector<pcl::index_t> Indices;
	std::vector<float> DistanceSet;
	m_pTree->radiusSearch(vInputPoint, LargeScaleRadius, Indices, DistanceSet);
	
	auto SinglePointFeature = __calcPointCloudNormalComplexity(Indices);
	return 1.0 - static_cast<double>(abs(SinglePointFeature - m_AverageDon));
}

//*****************************************************************
//FUNCTION: 
float CNormalComplexity::__calcPointCloudNormalComplexity(const std::vector<pcl::index_t>& vPointIndices)
{
	const auto& CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();
	
	const double SmallScaleRadius = *m_pConfig->getAttribute<double>("SMALL_SCALE_RADIUS");
	const double LargeScaleRadius = *m_pConfig->getAttribute<double>("LARGE_SCALE_RADIUS");

	pcl::PointCloud<pcl::PointXYZ>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (auto i : vPointIndices)
	{
		const auto& Position = CloudScene.getPositionAt(i);
		pPointCloud->emplace_back(Position.x(), Position.y(), Position.z());
	}
	
	pcl::PointCloud<pcl::PointNormal>::Ptr pSmallScalePointCloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr pLargeScalePointCloud(new pcl::PointCloud<pcl::PointNormal>);

	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> NormalEstimation;
	NormalEstimation.setInputCloud(pPointCloud);

	NormalEstimation.setRadiusSearch(SmallScaleRadius);
	NormalEstimation.compute(*pSmallScalePointCloud);

	NormalEstimation.setRadiusSearch(LargeScaleRadius);
	NormalEstimation.compute(*pLargeScalePointCloud);

	pcl::PointCloud<pcl::PointNormal> DonCloud;
	pcl::copyPointCloud(*pPointCloud, DonCloud);
	
	pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> DonEstimation;
	DonEstimation.setInputCloud(pPointCloud);
	DonEstimation.setNormalScaleSmall(pSmallScalePointCloud);
	DonEstimation.setNormalScaleLarge(pLargeScalePointCloud);
	if (!DonEstimation.initCompute())
		throw std::runtime_error("Don initCompute error");
	DonEstimation.computeFeature(DonCloud);

	auto DonMap = DonCloud.getMatrixXfMap(1, sizeof(pcl::PointNormal) / sizeof(float), offsetof(pcl::PointNormal, curvature) / sizeof(float));
	return DonMap.mean();
}
