#include "pch.h"
#include "PointCloudVisualizer.h"
#include "InteractionCallback.h"
#include "AutoRetouchInterface.h"

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
void CPointCloudVisualizer::init(pcl::PointCloud<pcl::PointSurfel>* vPointCloud)
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
void CPointCloudVisualizer::reset(pcl::PointCloud<pcl::PointSurfel>* vPointCloud)
{
	_ASSERTE(vPointCloud);
	m_pSceneCloud = vPointCloud;
	
}

//*****************************************************************
//FUNCTION: 
void CPointCloudVisualizer::refresh(bool vResetCamera)
{
	_ASSERTE(!m_pSceneCloud->empty());

	m_pPCLVisualizer->removeAllPointClouds();

	std::vector<AutoRetouch::EPointLabel> GlobalLabel;
	AutoRetouch::hiveGetGlobalPointLabelSet(GlobalLabel);
	
	_ASSERTE(GlobalLabel.size() == m_pSceneCloud->size());

	pcl::PointCloud<pcl::PointSurfel> Cloud2Show;
	Cloud2Show.resize(m_pSceneCloud->size());
	std::memcpy(Cloud2Show.data(), m_pSceneCloud->data(), m_pSceneCloud->size() * sizeof(pcl::PointSurfel));

	for (int i = 0; i < m_pSceneCloud->size(); i++)
	{
		switch (GlobalLabel[i])
		{
		case AutoRetouch::EPointLabel::DISCARDED:
		{
			Cloud2Show.points[i].a = 0;
			break;
		}
		case AutoRetouch::EPointLabel::UNWANTED:
		{
			unsigned char StandardRed[4] = { 0, 0, 255, 255 };	//gbr
			std::memcpy(&Cloud2Show.points[i].rgba, StandardRed, sizeof(StandardRed));
			break;
		}
		case AutoRetouch::EPointLabel::KEPT:
		{
			unsigned char StandardBlue[4] = { 255, 0, 0, 255 };
			std::memcpy(&Cloud2Show.points[i].rgba, StandardBlue, sizeof(StandardBlue));
			break;
		}
		case AutoRetouch::EPointLabel::FILLED:
		{
			unsigned char StandardGreen[4] = { 0, 255, 0, 255 };
			std::memcpy(&Cloud2Show.points[i].rgba, StandardGreen, sizeof(StandardGreen));
			break;
		}
		case AutoRetouch::EPointLabel::UNDETERMINED:
		{
			Cloud2Show.points[i].a = 255;
			break;
		}
		}
	}

	auto pCloud = Cloud2Show.makeShared();
	pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointSurfel> RGBAColor(pCloud);
	m_pPCLVisualizer->addPointCloud<pcl::PointSurfel>(pCloud, RGBAColor, "Cloud2Show");
	m_pPCLVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud2Show");

	if (vResetCamera)
		m_pPCLVisualizer->resetCamera();
	else
		m_pPCLVisualizer->updateCamera();
}