#include "pch.h"
#include "PrecomputeManager.h"
#include <fstream>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/utility.hpp>

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
void CPrecomputeManager::precompute()
{
	for (auto pFunc : m_PrecomputeList)
		pFunc();
}

const hiveConfig::CHiveConfig* CPrecomputeManager::getFeatureConfig(const std::string& vFeatureSig)
{
	for (auto i = 0; i < m_pClusterConfig->getNumSubconfig(); i++)
	{
		const hiveConfig::CHiveConfig* pConfig = m_pClusterConfig->getSubconfigAt(i);
		if (_IS_STR_IDENTICAL(pConfig->getSubconfigType(), std::string("FEATURE")))
		{
			std::optional<std::string> FeatureSig = pConfig->getAttribute<std::string>("SIG");
			_ASSERTE(FeatureSig.has_value());

			if (FeatureSig == vFeatureSig)
				return pConfig;
		}
	}
	return nullptr;
}

//*****************************************************************
//FUNCTION: 
void CPrecomputeManager::clear()
{
	m_PrecomputeList.clear();
}


