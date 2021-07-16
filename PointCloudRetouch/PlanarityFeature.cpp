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
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr pDeterminantCloud(new pcl::PointCloud<pcl::PointXYZ>);
	CloudScene.dumpPointCloud(vDeterminantPointSet, *pDeterminantCloud);
	
	auto Plane = __fitPlane(pDeterminantCloud);
	if (Plane.norm() < 1.0f)
		return 0.0;
	else
		m_Plane = Plane;
	m_Peak = __computePeakDistance(pDeterminantCloud, m_Plane);
	
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
	float Distance = m_Plane.dot(Position);

	if (Distance >= m_Peak.second || Distance <= m_Peak.first)
		return 0;

	auto m_Tolerance = m_pConfig->getAttribute<float>("DISTANCE_TOLERANCE").value();

	if (m_Peak.first * m_Tolerance <= Distance && Distance <= m_Peak.second * m_Tolerance)
		return 1;

	if (Distance < 0)
	{
		Distance -= m_Peak.first * m_Tolerance;
		Distance /= m_Peak.first * (m_Tolerance - 1.0f);
	}
	else
	{
		Distance -= m_Peak.second * m_Tolerance;
		Distance /= m_Peak.second * (1.0f - m_Tolerance);
	}
	return { pow(Distance, 4.0f) - 2.0f * pow(Distance, 2.0f) + 1.0f };
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
Eigen::Vector4f CPlanarityFeature::__fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr vCloud) const
{
	_ASSERTE(m_pConfig);
	Eigen::VectorXf Coeff;
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr ModelPlane
		(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(vCloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> Ransac(ModelPlane);
	
	Ransac.setDistanceThreshold(m_pConfig->getAttribute<float>("DISTANCE_THRESHOLD").value());
	Ransac.computeModel();
	Ransac.getModelCoefficients(Coeff);
	if (!Coeff.size())
		return { 0, 0, 0, 0 };
	const Eigen::Vector3f Normal(Coeff.x(), Coeff.y(), Coeff.z());
	//TODO: move to config
	const Eigen::Vector3f Up(0.0f, 0.0f, 1.0f);
	if (Normal.dot(Up) < 0.0f)
		Coeff *= -1.0f;

	return Coeff / Normal.norm();
}

//*****************************************************************
//FUNCTION: 
std::pair<float, float> CPlanarityFeature::__computePeakDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr vCloud, Eigen::Vector4f vPlane)
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
