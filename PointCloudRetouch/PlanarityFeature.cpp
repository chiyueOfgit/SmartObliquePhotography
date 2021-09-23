#include "pch.h"
#include "PlanarityFeature.h"
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>

using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CPlanarityFeature, KEYWORD::PLANARITY_FEATURE)

//*****************************************************************
//FUNCTION: 
void  CPlanarityFeature::initV(const hiveConfig::CHiveConfig* vFeatureConfig)
{
	_ASSERTE(vFeatureConfig);
	m_pConfig = vFeatureConfig;

	m_DistanceThreshold = m_pConfig->getAttribute<float>("DISTANCE_THRESHOLD").value();
	m_Tolerance = m_pConfig->getAttribute<float>("DISTANCE_TOLERANCE").value();
}

//*****************************************************************
//FUNCTION: 
double CPlanarityFeature::generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter)
{
	if (vDeterminantPointSet.empty() || vValidationSet.empty())
		return 0.0;

	const auto& CloudScene = CPointCloudRetouchManager::getInstance()->getScene();
	
	const pcl::PointCloud<pcl::PointXYZ>::Ptr pDeterminantCloud(new pcl::PointCloud<pcl::PointXYZ>);
	CloudScene.dumpPointCloud(vDeterminantPointSet, *pDeterminantCloud);

	m_Plane = fitPlane(pDeterminantCloud, m_DistanceThreshold, { 0.0f, 0.0f, 1.0f });
	if (m_Plane.squaredNorm() < 0.5f)
		return 0.0;
		
	double SumMatch = 0.0;
	for (auto& i : vValidationSet)
	{
		SumMatch += evaluateFeatureMatchFactorV(i);
	}

	hiveEventLogger::hiveOutputEvent((_FORMAT_STR1("m_Plane: %1%", m_Plane)));
	//DEBUG
	//hiveEventLogger::hiveOutputEvent((_FORMAT_STR1("Planarity Feature's Weight is: %1%\n", SumMatch / vValidationSet.size())));
	//DEBUG

	return SumMatch / vValidationSet.size();
}

//*****************************************************************
//FUNCTION: 
double CPlanarityFeature::evaluateFeatureMatchFactorV(pcl::index_t vInputPoint)
{
	if (m_Plane.squaredNorm() < 0.5f)
		return 0.0;
	
	const auto& Point = CPointCloudRetouchManager::getInstance()->getScene();
	const auto Distance = abs(m_Plane.dot(Point.getPositionAt(vInputPoint)));
	const auto NormalDot = abs(m_Plane.dot(Point.getNormalAt(vInputPoint)));

	if (Distance >= m_DistanceThreshold)
		return 0;
	else if (Distance >= m_DistanceThreshold * m_Tolerance)
		return NormalDot * smoothAttenuation(m_DistanceThreshold * m_Tolerance, m_DistanceThreshold, Distance);
	else
		return NormalDot;
}

//*****************************************************************
//FUNCTION: 
std::string CPlanarityFeature::outputDebugInfosV(pcl::index_t vIndex) const
{
	std::string Infos;
	Infos += "\nPlanarity Feature:\n";
	Infos += _FORMAT_STR3("Plane's Normal is: %1%, %2%, %3%\n", m_Plane.x(), m_Plane.y(), m_Plane.z());
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
float CPlanarityFeature::smoothAttenuation(float vFrom, float vTo, float vX)
{
	auto Factor = (vX - vFrom) / (vTo - vFrom);

	if (Factor >= 1 || Factor <= 0)
		return 0;

	//x^4 - 2 * x^2 + 1 
	Factor *= Factor;
	return Factor * (Factor - 2) + 1;
}
