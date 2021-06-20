#include "pch.h"
#include "Task.h"

using namespace hiveObliquePhotography::AutoRetouch;

ITask::~ITask()
{

}

//*****************************************************************
//FUNCTION: 
bool ITask::init(const hiveConfig::CHiveConfig* vTaskConfig, CGlobalPointLabelSet *vGlobapPointLabelSet)
{
	_ASSERTE(vTaskConfig && vGlobapPointLabelSet);
	_ASSERTE(!m_pTaskConfig && !m_pGlobalPointLabelSet && m_ClassifierSet.empty());

	m_pTaskConfig = vTaskConfig;
	m_pGlobalPointLabelSet = vGlobapPointLabelSet;

	return true;
}

