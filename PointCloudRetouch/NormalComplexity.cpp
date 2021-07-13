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

	pcl::PointCloud<pcl::PointXYZ>::Ptr pDeterminantPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (auto i : vDeterminantPointSet)
	{
		const auto& Position = CloudScene.getPositionAt(i);
		pDeterminantPointCloud->emplace_back(Position.x(), Position.y(), Position.z());
	}
	m_AverageDon = __calcPointCloudNormalComplexity(pDeterminantPointCloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pValidationPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (auto i : vValidationSet)
	{
		const auto& Position = CloudScene.getPositionAt(i);
		pValidationPointCloud->emplace_back(Position.x(), Position.y(), Position.z());
	}
	auto ValidationFeature = __calcPointCloudNormalComplexity(pValidationPointCloud);
	
	return 1.0 - static_cast<double>(abs(ValidationFeature - m_AverageDon));
}

//*****************************************************************
//FUNCTION: 
double CNormalComplexity::evaluateFeatureMatchFactorV(pcl::index_t vInputPoint)
{
	auto SinglePointFeature =__calcSinglePointNormalComplexity(vInputPoint);

	return 1.0 - static_cast<double>(abs(SinglePointFeature - m_AverageDon));
}

//*****************************************************************
//FUNCTION: 
float CNormalComplexity::__calcSinglePointNormalComplexity(pcl::index_t vIndex)
{
	const auto& CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();

	const double SmallScaleRadius = *m_pConfig->getAttribute<double>("SMALL_SCALE_RADIUS");
	const double LargeScaleRadius = *m_pConfig->getAttribute<double>("LARGE_SCALE_RADIUS");
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr pWholePointCloud(new pcl::PointCloud<pcl::PointXYZ>);
	for (size_t i = 0; i < CloudScene.getNumPoint(); i++)
	{
		const auto& Position = CloudScene.getPositionAt(i);
		pWholePointCloud->emplace_back(Position.x(), Position.y(), Position.z());
	}
	
	pcl::PointCloud<pcl::PointNormal> SmallScalePointCloud, LargeScalePointCloud;
	Eigen::Vector3f SmallScaleNormal, LargeScaleNormal;

	pcl::IndicesPtr pSingleIndex(new pcl::Indices);
	pSingleIndex->push_back(vIndex);
	
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> NormalEstimation;
	NormalEstimation.setInputCloud(pWholePointCloud);
	NormalEstimation.setIndices(pSingleIndex);
	
	NormalEstimation.setRadiusSearch(SmallScaleRadius);
	NormalEstimation.compute(SmallScalePointCloud);
	
	NormalEstimation.setRadiusSearch(LargeScaleRadius);
	NormalEstimation.compute(LargeScalePointCloud);
	
	SmallScaleNormal = SmallScalePointCloud.back().getNormalVector3fMap();
	LargeScaleNormal = LargeScalePointCloud.back().getNormalVector3fMap();

	return ((SmallScaleNormal - LargeScaleNormal) / 2).norm();
}

//*****************************************************************
//FUNCTION: 
float CNormalComplexity::__calcPointCloudNormalComplexity(pcl::PointCloud<pcl::PointXYZ>::Ptr vCloud)
{
	const double SmallScaleRadius = *m_pConfig->getAttribute<double>("SMALL_SCALE_RADIUS");
	const double LargeScaleRadius = *m_pConfig->getAttribute<double>("LARGE_SCALE_RADIUS");
	
	pcl::PointCloud<pcl::PointNormal>::Ptr pSmallScalePointCloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr pLargeScalePointCloud(new pcl::PointCloud<pcl::PointNormal>);

	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> NormalEstimation;
	NormalEstimation.setInputCloud(vCloud);

	NormalEstimation.setRadiusSearch(SmallScaleRadius);
	NormalEstimation.compute(*pSmallScalePointCloud);

	NormalEstimation.setRadiusSearch(LargeScaleRadius);
	NormalEstimation.compute(*pLargeScalePointCloud);

	pcl::PointCloud<pcl::PointNormal> DonCloud;
	pcl::copyPointCloud(*vCloud, DonCloud);
	
	pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> DonEstimation;
	DonEstimation.setInputCloud(vCloud);
	DonEstimation.setNormalScaleSmall(pSmallScalePointCloud);
	DonEstimation.setNormalScaleLarge(pLargeScalePointCloud);
	if (!DonEstimation.initCompute())
		throw std::runtime_error("Don initCompute error");
	DonEstimation.computeFeature(DonCloud);

	auto DonMap = DonCloud.getMatrixXfMap(1, sizeof(pcl::PointNormal) / sizeof(float), offsetof(pcl::PointNormal, curvature) / sizeof(float));
	return DonMap.mean();
}
