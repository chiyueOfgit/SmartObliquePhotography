#include "pch.h"
#include "PrecomputeManager.h"


using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
void CPrecomputeManager::registerPrecomputeFunction(std::function<bool()> vPrecomputeFunc)
{
	m_PrecomputeList.push_back(vPrecomputeFunc);
}

//*****************************************************************
//FUNCTION: 
void CPrecomputeManager::runAllPrecompute()
{
	for (auto pFunc : m_PrecomputeList)
		pFunc();
}

//*****************************************************************
//FUNCTION: 
void CPrecomputeManager::clear()
{
	m_PrecomputeList.clear();
}
