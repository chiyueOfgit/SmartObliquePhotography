#include "pch.h"
#include "NormalComplexity.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/features/don.h"
#include "EuclideanNeighborhoodBuilder.h"
#include "pcl/io/pcd_io.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CNormalComplexity, KEYWORD::NORMAL_COMPLEXITY)

//*****************************************************************
//FUNCTION: 
void  CNormalComplexity::initV(const hiveConfig::CHiveConfig* vFeatureConfig)
{
	_ASSERTE(vFeatureConfig);
	m_pConfig = vFeatureConfig;

	std::optional<std::string> PrecomputeCloudPath = m_pConfig->getAttribute<std::string>("PRECOMPUTE_CLOUD_PATH");
	PointCloud_t pPrecomputeCloud;

	std::string FileName = hiveUtility::hiveLocateFile(PrecomputeCloudPath.value());
	PointCloud_t::Ptr pTempCloud(new PointCloud_t);
	if (PrecomputeCloudPath.has_value() && !FileName.empty() && pcl::io::loadPCDFile(PrecomputeCloudPath.value(), *pTempCloud) == 0)
	{
		hiveEventLogger::hiveOutputEvent("successfully load precomputed normal complexity.");
		
		m_pNormalComplexity = pTempCloud;
	}
	else
	{
		const auto& CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();

		pcl::PointCloud<pcl::PointXYZ>::Ptr pPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
		for (size_t i = 0; i < CloudScene.getNumPoint(); i++)
		{
			const auto& Position = CloudScene.getPositionAt(i);
			pPointCloud->emplace_back(Position.x(), Position.y(), Position.z());
		}

		if (pPointCloud->isOrganized())
			m_pTree.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
		else
			m_pTree.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));
		m_pTree->setInputCloud(pPointCloud);
	}
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
	return 1.0 - abs(__calcSinglePointNormalComplexity(vInputPoint) - m_AverageDon);
}

//*****************************************************************
//FUNCTION: 
std::string CNormalComplexity::outputDebugInfosV(pcl::index_t vIndex) const
{
	std::string Infos;
	Infos += "\nNormal Featrue:\n";
	Infos += _FORMAT_STR1("Average Normal Complexity is: %1%\n", m_AverageDon);
	Infos += _FORMAT_STR1("Point's Normal Complexity is: %1%\n", const_cast<CNormalComplexity*>(this)->__calcSinglePointNormalComplexity(vIndex));
	Infos += _FORMAT_STR1("Similarity is: %1%\n", const_cast<CNormalComplexity*>(this)->evaluateFeatureMatchFactorV(vIndex));

	return Infos;
}

//*****************************************************************
//FUNCTION: 
bool CNormalComplexity::__precomputeSceneCloudNormalComplexity()
{
	
}

//*****************************************************************
//FUNCTION: 
double CNormalComplexity::__calcPointCloudNormalComplexity(const std::vector<pcl::index_t>& vPointIndices)
{
	double Sum = 0.0;
	for (auto& i : vPointIndices)
		Sum += __calcSinglePointNormalComplexity(i);
	return Sum / vPointIndices.size();
}

//*****************************************************************
//FUNCTION: 
double CNormalComplexity::__calcSinglePointNormalComplexity(pcl::index_t vInputPoint) const
{
	if (m_pNormalComplexity)

	const auto& CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();
	const double Radius = *m_pConfig->getAttribute<double>("LARGE_SCALE_RADIUS");
	
	pcl::Indices Neighborhood;
	std::vector<float> DistanceSet;
	m_pTree->radiusSearch(vInputPoint, Radius, Neighborhood, DistanceSet);

	double MinD = DBL_MAX;
	double MaxD = -DBL_MAX;
	double MeanD = 0.0;
	const auto& Normal = CloudScene.getNormalAt(vInputPoint);
	//Normal.normalize();
	for (auto& NeighborIndex : Neighborhood)
	{		
		const double D = CloudScene.getPositionAt(NeighborIndex).dot(Normal);
		MeanD += D;
		if (MinD > D)
			MinD = D;
		if (MaxD < D)
			MaxD = D;
	}
	MeanD /= Neighborhood.size();
	
	double Complexity = std::min(abs(MinD - MeanD), abs(MaxD - MeanD));
	
	//double Complexity = (MaxD - MinD) / 2;
	Complexity /= Radius;

	if (Complexity > 1)
		return 1;
	
	return Complexity;
}