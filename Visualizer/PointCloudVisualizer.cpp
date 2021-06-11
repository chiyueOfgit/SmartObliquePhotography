#include "pch.h"
#include "PointCloudVisualizer.h"
#include "InteractionCallback.h"
#include "AutoRetouchInterface.h"
#include <omp.h>

#define RECORD_TIME_BEGIN clock_t StartTime, FinishTime;\
StartTime = clock();

#define RECORD_TIME_END(Name) FinishTime = clock();\
std::cout << "\n" << #Name << "花费时间: " << (int)(FinishTime - StartTime) << " ms\n";

using namespace hiveObliquePhotography::Visualization;

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
void CPointCloudVisualizer::init(pcl::PointCloud<pcl::PointSurfel>::Ptr vPointCloud, bool vIsInQt)
{
	_ASSERTE(vPointCloud);
	m_pSceneCloud = vPointCloud;

	m_pPCLVisualizer = new pcl::visualization::PCLVisualizer("Visualizer", !vIsInQt);
	m_pCallback = new CInteractionCallback(m_pPCLVisualizer);
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
	RECORD_TIME_BEGIN

	_ASSERTE(!m_pSceneCloud->empty());

	m_pPCLVisualizer->removeAllPointClouds();

	std::vector<AutoRetouch::EPointLabel> GlobalLabel;
	AutoRetouch::hiveGetGlobalPointLabelSet(GlobalLabel);
	
	_ASSERTE(GlobalLabel.size() == m_pSceneCloud->size());

	pcl::PointCloud<pcl::PointSurfel>::Ptr pCloud2Show(new pcl::PointCloud<pcl::PointSurfel>);
	pCloud2Show->resize(m_pSceneCloud->size());
	std::memcpy(pCloud2Show->data(), m_pSceneCloud->data(), m_pSceneCloud->size() * sizeof(pcl::PointSurfel));
	
	#pragma omp parallel for
	for (int i = 0; i < m_pSceneCloud->size(); i++)
	{
		switch (GlobalLabel[i])
		{
		case AutoRetouch::EPointLabel::DISCARDED:
		{
			pCloud2Show->points[i].a = 0;
			break;
		}
		case AutoRetouch::EPointLabel::UNWANTED:
		{
			unsigned char StandardRed[4] = { 0, 0, 255, 255 };	//gbr
			std::memcpy(&pCloud2Show->points[i].rgba, StandardRed, sizeof(StandardRed));
			break;
		}
		case AutoRetouch::EPointLabel::KEPT:
		{
			unsigned char StandardBlue[4] = { 255, 0, 0, 255 };
			std::memcpy(&pCloud2Show->points[i].rgba, StandardBlue, sizeof(StandardBlue));
			break;
		}
		case AutoRetouch::EPointLabel::FILLED:
		{
			unsigned char StandardGreen[4] = { 0, 255, 0, 255 };
			std::memcpy(&pCloud2Show->points[i].rgba, StandardGreen, sizeof(StandardGreen));
			break;
		}
		case AutoRetouch::EPointLabel::UNDETERMINED:
		{
			pCloud2Show->points[i].a = 255;
			break;
		}
		}
	}

	pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointSurfel> RGBAColor(pCloud2Show);
	m_pPCLVisualizer->addPointCloud<pcl::PointSurfel>(pCloud2Show, RGBAColor, "Cloud2Show");
	m_pPCLVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud2Show");

	if (vResetCamera)
		m_pPCLVisualizer->resetCamera();
	else
		m_pPCLVisualizer->updateCamera();

	RECORD_TIME_END(显示)
}

void CPointCloudVisualizer::run()
{
	while (!m_pPCLVisualizer->wasStopped())
	{
		m_pPCLVisualizer->spinOnce(16);
	}
}