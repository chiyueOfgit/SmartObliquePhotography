#include "pch.h"
#include "NormalComplexity.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/features/don.h"
#include "EuclideanNeighborhoodBuilder.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CNormalComplexity, KEYWORD::NORMAL_COMPLEXITY)

//*****************************************************************
//FUNCTION: 
bool CNormalComplexity::onProductCreatedV(const hiveConfig::CHiveConfig* vFeatureConfig)
{
	_ASSERTE(vFeatureConfig);
	m_pConfig = vFeatureConfig;

	const auto& CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();

	const double LargeScaleRadius = *m_pConfig->getAttribute<double>("LARGE_SCALE_RADIUS");

	pcl::PointCloud<pcl::PointNormal>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointNormal>);
	for (size_t i = 0; i < CloudScene.getNumPoint(); i++)
	{
		pcl::PointNormal Point;
		const auto& Position = CloudScene.getPositionAt(i);
		Point.x = Position.x();
		Point.y = Position.y();
		Point.z = Position.z();
		const auto& Normal = CloudScene.getNormalAt(i);
		Point.normal_x = Normal.x();
		Point.normal_y = Normal.y();
		Point.normal_z = Normal.z();
		
		pPointCloud->push_back(Point);
	}
	pcl::PointCloud<pcl::PointNormal>::Ptr pLargeScalePointCloud(new pcl::PointCloud<pcl::PointNormal>);

	pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal> NormalEstimation;
	NormalEstimation.setInputCloud(pPointCloud);

	NormalEstimation.setRadiusSearch(LargeScaleRadius);
	NormalEstimation.compute(*pLargeScalePointCloud);

	pcl::copyPointCloud(*pPointCloud, m_DonCloud);
	
	pcl::DifferenceOfNormalsEstimation<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal> DonEstimation;
	DonEstimation.setInputCloud(pPointCloud);
	DonEstimation.setNormalScaleSmall(pPointCloud);
	DonEstimation.setNormalScaleLarge(pLargeScalePointCloud);
	if (!DonEstimation.initCompute())
		throw std::runtime_error("Don initCompute error");
	DonEstimation.computeFeature(m_DonCloud);
	
	return true;
}

//*****************************************************************
//FUNCTION: 
double CNormalComplexity::generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter)
{
	const auto& CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();
	
	if (vDeterminantPointSet.empty() || vValidationSet.empty())
		return 0.0;

	m_AverageDon = __calcPointCloudNormalComplexity(vDeterminantPointSet);

	double Score = 0.0;
	for (auto& i : vValidationSet)
		Score += evaluateFeatureMatchFactorV(i);
	Score /= vValidationSet.size();
	return Score;
}

//*****************************************************************
//FUNCTION: 
double CNormalComplexity::evaluateFeatureMatchFactorV(pcl::index_t vInputPoint)
{
	return 1.0 - static_cast<double>(abs(__calcSinglePointNormalComplexity(vInputPoint) - m_AverageDon));
}

//*****************************************************************
//FUNCTION: 
std::string CNormalComplexity::outputDebugInfosV(pcl::index_t vIndex) const
{
	std::string Infos;
	Infos += "\nNormal Feature:\n";
	Infos += _FORMAT_STR1("Average Normal Complexity is: %1%\n", m_AverageDon);
	Infos += _FORMAT_STR1("Point's Normal Complexity is: %1%\n", const_cast<CNormalComplexity*>(this)->__calcSinglePointNormalComplexity(vIndex));
	Infos += _FORMAT_STR1("Similarity is: %1%\n", const_cast<CNormalComplexity*>(this)->evaluateFeatureMatchFactorV(vIndex));

	return Infos;
}

//*****************************************************************
//FUNCTION: 
float CNormalComplexity::__calcPointCloudNormalComplexity(const std::vector<pcl::index_t>& vPointIndices)
{
	double Sum = 0.0;
	for (auto& i : vPointIndices)
		Sum += m_DonCloud.at(i).curvature;

	return Sum / vPointIndices.size();
}

//*****************************************************************
//FUNCTION: 
float CNormalComplexity::__calcSinglePointNormalComplexity(pcl::index_t vInputPoint)
{
	return m_DonCloud.at(vInputPoint).curvature;
}