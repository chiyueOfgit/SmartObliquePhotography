#include "pch.h"
#include "PointCloudVisualizer.h"
#include "InteractionCallback.h"
//#include "PointCloudAutoRetouchScene.h"

using namespace hiveObliquePhotography::visualizer;

CPointCloudVisualizer::CPointCloudVisualizer()
{
}

CPointCloudVisualizer::~CPointCloudVisualizer()
{
	delete m_pPCLVisualizer;
	delete m_pCallback;
}

//*****************************************************************
//FUNCTION: 
void CPointCloudVisualizer::init(pcl::PointCloud<pcl::PointSurfel>::Ptr vPointCloud)
{
	_ASSERTE(vPointCloud);
	m_pSceneCloud = vPointCloud;

	m_pPCLVisualizer = new pcl::visualization::PCLVisualizer("Visualizer", true);
	_ASSERTE(m_pPCLVisualizer);

	m_pCallback = new CInteractionCallback(m_pPCLVisualizer);
	_ASSERTE(m_pCallback);

	m_pPCLVisualizer->setBackgroundColor(0.2, 0.2, 0.2);

}

//*****************************************************************
//FUNCTION: 
void CPointCloudVisualizer::reset(pcl::PointCloud<pcl::PointSurfel>::Ptr vPointCloud)
{
	_ASSERTE(vPointCloud);
	m_pSceneCloud = vPointCloud;
	
}

//*****************************************************************
//FUNCTION: 
void CPointCloudVisualizer::refresh(bool vResetCamera)
{
	m_pPCLVisualizer->removeAllPointClouds();

	m_pPCLVisualizer->addPointCloud<pcl::PointSurfel>(m_pSceneCloud);

	if (vResetCamera)
		m_pPCLVisualizer->resetCamera();
	else
		m_pPCLVisualizer->updateCamera();
}