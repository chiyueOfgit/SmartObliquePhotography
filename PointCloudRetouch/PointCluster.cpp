#include "pch.h"
#include "PointCluster.h"
#include "Feature.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
double CPointCluster::evaluateProbability(pcl::index_t vInputPoint) const
{
	if (m_FeatureSet.empty() || m_FeatureWeightSet.empty())
		_THROW_RUNTIME_ERROR("Point cluster is not uninitialized");
	
	if (vInputPoint < 0 || vInputPoint > CPointCloudRetouchManager::getInstance()->getRetouchScene().getNumPoint())
		_THROW_RUNTIME_ERROR("Index is out of range");
	
	double Probability = 0.0;
	double SumWeight = 0.0;
	for (auto i = 0; i < m_FeatureSet.size(); i++)
	{
		if (m_FeatureWeightSet[i] == 0)
			continue;
		
		Probability += m_FeatureWeightSet[i] * m_FeatureSet[i]->evaluateFeatureMatchFactorV(vInputPoint);
		SumWeight += m_FeatureWeightSet[i];
	}
	Probability /= SumWeight;

	_ASSERTE((Probability >= 0) && (Probability <= 1));
	return Probability;
}

//*****************************************************************
//FUNCTION: 
bool CPointCluster::init(const hiveConfig::CHiveConfig* vConfig, std::uint32_t vClusterCenter, EPointLabel vLabel, const std::vector<pcl::index_t>& vFeatureGenerationSet, const std::vector<pcl::index_t>& vValidationSet, std::uint32_t vCreationTimestamp)
{
	_ASSERTE(vConfig && !vFeatureGenerationSet.empty() && !vValidationSet.empty() && !m_pConfig);
	m_pConfig = vConfig;

	__createFeatureObjectSet();
	if (m_FeatureSet.empty())
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to initialize cluster [%1%] because no feature object is created.", m_pConfig->getName()));
		m_pConfig = nullptr;
		return false;
	}

	m_CreationTimestamp = vCreationTimestamp;
	m_ClusterCenter = vClusterCenter;
	m_Label = vLabel;
	m_ClusterCoreRegion = vFeatureGenerationSet;  //·¢Éústd::vector¿½±´

	_ASSERTE(m_pConfig);
	{
		std::optional<float> ExpectProbability = m_pConfig->getAttribute<float>("EXPECT_PROBABILITY");
		if (ExpectProbability.has_value())
			m_ExpectProbability = ExpectProbability.value();
	}

	__createFeatureObjectSet();

	for (auto e : m_FeatureSet)
	{
		m_FeatureWeightSet.emplace_back(e->generateFeatureV(m_ClusterCoreRegion, vValidationSet, m_ClusterCenter));
	}

	return true;
}

//*****************************************************************
//FUNCTION: 
void CPointCluster::__createFeatureObjectSet()
{
	for (auto i = 0; i < m_pConfig->getNumSubconfig(); i++)
	{
		const hiveConfig::CHiveConfig* pConfig = m_pConfig->getSubconfigAt(i);
		if (_IS_STR_IDENTICAL(pConfig->getSubconfigType(), std::string("FEATURE")))
		{
			auto createFeatureIfDefined = [&](const std::string& vFeatureName, const std::string& vClusterName)->bool
			{
				if (_IS_STR_IDENTICAL(pConfig->getName(), vFeatureName))
				{
					std::optional<std::string> FeatureSig = pConfig->getAttribute<std::string>("SIG");
					_ASSERTE(FeatureSig.has_value());
					auto pFeature = hiveDesignPattern::hiveGetOrCreateProduct<IFeature>(FeatureSig.value(), pConfig);
					if (pFeature)
						m_FeatureSet.push_back(pFeature);
					else
						_HIVE_OUTPUT_WARNING(_FORMAT_STR2("The feature [%1%] defined in point cluster [%2%] is ignored due to the failure of creating feature object.", FeatureSig.value(), vClusterName));
					return true;
				}
				else
					return false;
			};
			
			if (createFeatureIfDefined("PlanarFeature", m_pConfig->getName())) continue;
			if (createFeatureIfDefined("VFHFeature", m_pConfig->getName())) continue;
			if (createFeatureIfDefined("ColorFeature", m_pConfig->getName())) continue;

			_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Unknown feature [%1%].", pConfig->getName()));
		}
	}
}

//*****************************************************************
//FUNCTION: 
bool CPointCluster::isBelongingTo(double vProbability) const
{
	return vProbability >= m_ExpectProbability ? true : false;
}