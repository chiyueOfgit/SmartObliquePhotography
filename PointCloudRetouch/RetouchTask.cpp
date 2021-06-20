#include "pch.h"
#include "RetouchTask.h"
#include "PointClusterExpander.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
bool CRetouchTask::init(const hiveConfig::CHiveConfig* vConfig)
{
	_ASSERTE(vConfig);
	m_pConfig = vConfig;

//TODO: 根据配置文件指定的分类器的签名，使用工厂模式来创建相应对象并初始化

	return true;
}

//*****************************************************************
//FUNCTION: 
const hiveConfig::CHiveConfig* CRetouchTask::getClusterConfig() const
{
	return m_pConfig->findSubconfigByName("CLUSTER");
}

//*****************************************************************
//FUNCTION: 
bool CRetouchTask::execute(const CPointCluster* vUserSpecifiedCluster)
{
	_ASSERTE(vUserSpecifiedCluster && m_pPointClusterExpander);

	return m_pPointClusterExpander->execute<CPointClusterExpander>(vUserSpecifiedCluster);
}