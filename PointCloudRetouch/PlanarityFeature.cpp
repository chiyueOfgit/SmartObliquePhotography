#include "pch.h"
#include "PlanarityFeature.h"
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>

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

//*****************************************************************
//FUNCTION: 
Eigen::Vector4f CPlanarityFeature::fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr vCloud, double vDistanceThreshold, const Eigen::Vector3f& vUp)
{
	Eigen::VectorXf Coeff;
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr ModelPlane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(vCloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> Ransac(ModelPlane);
	Ransac.setDistanceThreshold(vDistanceThreshold);
	Ransac.computeModel();
	Ransac.getModelCoefficients(Coeff);
	if (!Coeff.size())
		return { 0, 0, 0, 0 };
	const Eigen::Vector3f Normal(Coeff.x(), Coeff.y(), Coeff.z());
	if (Normal.dot(vUp) < 0.0f)
		Coeff *= -1.0f;
	return Coeff / Normal.norm();
}

//*****************************************************************
//FUNCTION: 
std::pair<float, float> CPlanarityFeature::computePeakDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr vCloud, const Eigen::Vector4f& vPlane)
{
	float MinDistance = FLT_MAX;
	float MaxDistance = -FLT_MAX;
	for (auto& i : *vCloud)
	{
		MinDistance = std::min(MinDistance, vPlane.dot(i.getVector4fMap()));
		MaxDistance = std::max(MaxDistance, vPlane.dot(i.getVector4fMap()));
	}

	return { MinDistance, MaxDistance };
}

//*****************************************************************
//FUNCTION: 
float CPlanarityFeature::smoothAttenuation(float vFrom, float vTo, float vX)
{
	auto Factor = (vX - vFrom) / (vTo - vFrom);

	if (Factor >= 1 || Factor <= 0)
		return 0;

	//x^4 - 2 * x^2 + 1 
	Factor *= Factor;
	return Factor * (Factor - 2) + 1;
}
