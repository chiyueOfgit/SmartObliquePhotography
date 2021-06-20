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

	for (auto i = 0; i < vConfig->getNumSubconfig(); i++)
	{
		const hiveConfig::CHiveConfig* pConfig = vConfig->getSubconfigAt(i);
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
	}
	
	return true;
}

//*****************************************************************
//FUNCTION: 
CPointCluster* CPointCloudRetouchManager::__generateInitialCluster(const std::vector<pcl::index_t>& vUserMarkedRegion, double vHardness, double vRadius, const Eigen::Vector3f& vCameraPos, const Eigen::Matrix4d& vPvMatrix, EPointLabel vTargetLabel)
{
	_ASSERTE(m_pConfig);
	_ASSERTE((vTargetLabel == EPointLabel::KEPT) || (vTargetLabel == EPointLabel::UNWANTED));

	const hiveConfig::CHiveConfig* pClusterConfig = (vTargetLabel == EPointLabel::KEPT) ? m_BackgroundMarker.getClusterConfig() : m_LitterMarker.getClusterConfig();
	_ASSERTE(pClusterConfig);

	return m_InitialClusterCreator.createInitialCluster(vUserMarkedRegion, vHardness, vRadius, vTargetLabel, vCameraPos, vPvMatrix, pClusterConfig);
}

//*****************************************************************
//FUNCTION: 
bool CPointCloudRetouchManager::executeMarker(const std::vector<pcl::index_t>& vUserMarkedRegion, double vHardness, double vRadius, const Eigen::Vector3f& vCameraPos, const Eigen::Matrix4d& vPvMatrix, EPointLabel vTargetLabel)
{
	_ASSERTE((vTargetLabel == EPointLabel::UNWANTED) || (vTargetLabel == EPointLabel::KEPT));

	try
	{
		CPointCluster* pInitCluster = __generateInitialCluster(vUserMarkedRegion, vHardness, vRadius, vCameraPos, vPvMatrix, vTargetLabel);
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
void CPointCloudRetouchManager::buildNeighborhood(pcl::index_t vSeed, std::vector<pcl::index_t>& voNeighborhood)
{
	m_pNeighborhoodBuilder->buildNeighborhood(vSeed, voNeighborhood);
}