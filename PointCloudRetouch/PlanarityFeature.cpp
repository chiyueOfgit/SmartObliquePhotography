#include "pch.h"
#include "PlanarityFeature.h"
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>
#include <pcl/sample_consensus/impl/ransac.hpp>

using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CPlanarityFeature, KEYWORD::PLANARITY_FEATURE)

//*****************************************************************
//FUNCTION: 
double CPlanarityFeature::generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter)
{
	if (vDeterminantPointSet.empty() || vValidationSet.empty())
		return 0.0;

	const auto& CloudScene = CPointCloudRetouchManager::getInstance()->getRetouchScene();
	
	const pcl::PointCloud<pcl::PointXYZ>::Ptr pDeterminantCloud(new pcl::PointCloud<pcl::PointXYZ>);
	CloudScene.dumpPointCloud(vDeterminantPointSet, *pDeterminantCloud);
	
	m_Plane = fitPlane(pDeterminantCloud, 0.4, { 0.0f, 0.0f, 1.0f });
	if (m_Plane.norm() < 1.0f)
		return 0.0;
	
	m_Peak = computePeakDistance(pDeterminantCloud, m_Plane);
	
	double SumMatch = 0.0;
	for (auto& i : vValidationSet)
	{
		SumMatch += evaluateFeatureMatchFactorV(i);
	}

	//DEBUG
	//hiveEventLogger::hiveOutputEvent((_FORMAT_STR1("Planarity Feature's Weight is: %1%\n", SumMatch / vValidationSet.size())));
	//DEBUG

	return SumMatch / vValidationSet.size();
}

//*****************************************************************
//FUNCTION: 
double CPlanarityFeature::evaluateFeatureMatchFactorV(pcl::index_t vInputPoint)
{
	const auto& Position = CPointCloudRetouchManager::getInstance()->getRetouchScene().getPositionAt(vInputPoint);
	const float Distance = m_Plane.dot(Position);

	const auto Tolerance = m_pConfig->getAttribute<float>("DISTANCE_TOLERANCE").value();
	if (Distance <= m_Peak.first || Distance >= m_Peak.second)
		return 0;
	else if (m_Peak.first * Tolerance <= Distance && Distance <= m_Peak.second * Tolerance)
		return 1;
	else if (Distance < 0)
		return smoothAttenuation(m_Peak.first * Tolerance, m_Peak.first, Distance);
	else
		return smoothAttenuation(m_Peak.second * Tolerance, m_Peak.second, Distance);
}

//*****************************************************************
//FUNCTION: 
std::string CPlanarityFeature::outputDebugInfosV(pcl::index_t vIndex) const
{
	std::string Infos;
	Infos += "\nPlanarity Feature:\n";
	Infos += _FORMAT_STR3("Plane's Normal is: %1%, %2%, %3%\n", m_Plane.x(), m_Plane.y(), m_Plane.z());
	Infos += _FORMAT_STR2("Plane's Peak is: %1%, %2%\n", m_Peak.first, m_Peak.second);
	Infos += _FORMAT_STR1("Similarity is: %1%\n", const_cast<CPlanarityFeature*>(this)->evaluateFeatureMatchFactorV(vIndex));

	return Infos;
}
