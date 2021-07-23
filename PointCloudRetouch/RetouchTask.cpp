#include "pch.h"
#include "RetouchTask.h"
#include "PointClusterExpander.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
bool CRetouchTask::init(const hiveConfig::CHiveConfig* vConfig)
{
	_ASSERTE(vConfig);

	try 
	{
		auto NumSubconfig = vConfig->getNumSubconfig();
	}
	catch (...)
	{
		throw "vConfig Error";
		return false;
	}

	m_pConfig = vConfig;
	m_pClusterConfig = nullptr;

	for (auto i = 0; i < vConfig->getNumSubconfig(); i++)
	{
		const hiveConfig::CHiveConfig* pConfig = vConfig->getSubconfigAt(i);
		if (_IS_STR_IDENTICAL(pConfig->getSubconfigType(), std::string("CLUSTER")))
		{
			if (!m_pClusterConfig)
			{
				m_pClusterConfig = pConfig;
			}
			else
			{
				_HIVE_OUTPUT_WARNING("It is NOT allowed to define cluster twice.");
			}
			continue;
		}
		if (_IS_STR_IDENTICAL(pConfig->getSubconfigType(), std::string("CLASSIFIER")))
		{
			std::optional<std::string> ClassifierSig = pConfig->getAttribute<std::string>("SIG");
			_ASSERTE(ClassifierSig.has_value());
			m_pPointClusterExpander = dynamic_cast<CPointClusterExpander*>(hiveDesignPattern::hiveCreateProduct<IPointClassifier>(ClassifierSig.value()));
			_HIVE_EARLY_RETURN(!m_pPointClusterExpander, _FORMAT_STR1("Fail to initialize retouch due to the failure of creating point classifier [%1%].", ClassifierSig.value()), false);
			continue;
		}
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Unknown subconfiguration type [%1%].", pConfig->getSubconfigType()));
	}

	_HIVE_EARLY_RETURN(!m_pPointClusterExpander, _FORMAT_STR1("Fail to initialize the retouch task [%1%] due to missing point classifier.", vConfig->getName()), false);

	return true;
}

void CRetouchTask::reset()
{
	m_pConfig = nullptr;
	m_pClusterConfig = nullptr;
}

//*****************************************************************
//FUNCTION: 
bool CRetouchTask::execute(const CPointCluster* vUserSpecifiedCluster) const
{
	_ASSERTE(vUserSpecifiedCluster && m_pPointClusterExpander);

	m_pPointClusterExpander->execute<CPointClusterExpander>(vUserSpecifiedCluster);

	return m_pPointClusterExpander->getExpandPoints().size();
}

//*****************************************************************
//FUNCTION: 
void CRetouchTask::dumpTaskMarkedPoints(std::vector<pcl::index_t>& voMarkedPoints) const
{
	voMarkedPoints = m_pPointClusterExpander->getExpandPoints();
}

