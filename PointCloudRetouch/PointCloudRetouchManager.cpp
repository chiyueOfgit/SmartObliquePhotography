#include "pch.h"
#include "PointCloudRetouchManager.h"
#include "PointCluster.h"
#include "NeighborhoodBuilder.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
bool CPointCloudRetouchManager::init(PointCloud_t::Ptr vPointCloud, const hiveConfig::CHiveConfig* vConfig)
{
	_ASSERTE(vPointCloud && vConfig);

	m_Scene.init(vPointCloud);
	m_PointLabelSet.init(m_Scene.getNumPoint());
	m_pConfig = vConfig;

	if (_IS_STR_IDENTICAL(vConfig->getSubconfigAt(0)->getSubconfigType(), std::string("POINT_CLOUD_RETOUCN_CONFIG")))
	{
		_ASSERTE(vConfig->getNumSubconfig());
		if (_IS_STR_IDENTICAL(vConfig->getSubconfigAt(0)->getName(), std::string("Retouch")))
			vConfig = vConfig->getSubconfigAt(0);
	}

	for (auto i = 0; i < vConfig->getNumSubconfig(); i++)
	{
		const hiveConfig::CHiveConfig* pConfig = vConfig->getSubconfigAt(i);
		if (_IS_STR_IDENTICAL(pConfig->getSubconfigType(), std::string("TASK")))
		{
			if (_IS_STR_IDENTICAL(pConfig->getName(), std::string("LitterMarker")))
			{
				m_LitterMarker.init(pConfig);
			}
			else
			{
				if (_IS_STR_IDENTICAL(pConfig->getName(), std::string("BackgroundMarker")))
				{
					m_BackgroundMarker.init(pConfig);
				}
				else
				{
					_HIVE_EARLY_RETURN(true, _FORMAT_STR1("Unexpeced subconfiguration [%1%].", pConfig->getName()), false);
				}
			}
			continue;
		}
		if (_IS_STR_IDENTICAL(pConfig->getSubconfigType(), std::string("NEIGHBOR_BUILDER")))
		{
			std::optional<std::string> NeighborhoodBuilderSig = pConfig->getAttribute<std::string>("SIG");
			_ASSERTE(NeighborhoodBuilderSig.has_value());
			m_pNeighborhoodBuilder = hiveDesignPattern::hiveCreateProduct<INeighborhoodBuilder>(NeighborhoodBuilderSig.value(), vPointCloud, &m_PointLabelSet);
			_HIVE_EARLY_RETURN(!m_pNeighborhoodBuilder, _FORMAT_STR1("Fail to initialize retouch due to the failure of creating  neighborhood builder [%1%].", NeighborhoodBuilderSig.value()), false);
			continue;
		}
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Unknown subconfiguration type [%1%].", pConfig->getSubconfigType()));
	}
	
	return true;
}

//*****************************************************************
//FUNCTION: 
CPointCluster* CPointCloudRetouchManager::__generateInitialCluster(const std::vector<pcl::index_t>& vUserMarkedRegion, double vHardness, double vRadius, const Eigen::Vector2f& vCenter, const Eigen::Matrix4d& vPvMatrix, const std::pair<float, float>& vWindowSize, EPointLabel vTargetLabel)
{
	_ASSERTE(m_pConfig);
	_ASSERTE((vTargetLabel == EPointLabel::KEPT) || (vTargetLabel == EPointLabel::UNWANTED));

	const hiveConfig::CHiveConfig* pClusterConfig = (vTargetLabel == EPointLabel::KEPT) ? m_BackgroundMarker.getClusterConfig() : m_LitterMarker.getClusterConfig();
	_ASSERTE(pClusterConfig);

	return m_InitialClusterCreator.createInitialCluster(vUserMarkedRegion, vHardness, vRadius, vTargetLabel, vCenter, vPvMatrix, vWindowSize, pClusterConfig);
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::PointCloudRetouch::CPointCloudRetouchManager::dumpPointLabel(std::vector<std::size_t>& voPointLabel) const
{
	auto NumPoints = m_Scene.getNumPoint();
	if (NumPoints > 0)
	{
		voPointLabel.clear();

		for (int i = 0; i < NumPoints; i++)
		{
			auto Label = m_PointLabelSet.getLabelAt(i);
			voPointLabel.push_back(static_cast<std::size_t>(Label));
		}

		return true;
	}
	else
		return false;
}

//*****************************************************************
//FUNCTION: 
bool CPointCloudRetouchManager::executeMarker(const std::vector<pcl::index_t>& vUserMarkedRegion, double vHardness, double vRadius, const Eigen::Vector2f& vCenter, const Eigen::Matrix4d& vPvMatrix, const std::pair<float, float>& vWindowSize, EPointLabel vTargetLabel)
{
	_ASSERTE((vTargetLabel == EPointLabel::UNWANTED) || (vTargetLabel == EPointLabel::KEPT));

	try
	{
		CPointCluster* pInitCluster = __generateInitialCluster(vUserMarkedRegion, vHardness, vRadius, vCenter, vPvMatrix, vWindowSize, vTargetLabel);
		_ASSERTE(pInitCluster);
		m_PointLabelSet.tagCoreRegion4Cluster(pInitCluster->getCoreRegion(), vTargetLabel, pInitCluster->getClusterIndex());

		if (vTargetLabel == EPointLabel::UNWANTED)
		{
			m_LitterMarker.execute(pInitCluster);
		}
		else
		{
			m_BackgroundMarker.execute(pInitCluster);
		}
		return true;
	}
	catch (std::runtime_error& e)
	{

	}
	catch (...)
	{

	}
	return false;
}

//*****************************************************************
//FUNCTION: 
void CPointCloudRetouchManager::buildNeighborhood(pcl::index_t vSeed, std::uint32_t vSeedClusterIndex, std::vector<pcl::index_t>& voNeighborhood)
{
	m_pNeighborhoodBuilder->buildNeighborhood(vSeed, vSeedClusterIndex, voNeighborhood);
}