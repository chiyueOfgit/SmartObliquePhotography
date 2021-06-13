#include "pch.h"
#include "InteractionCallback.h"
#include "PointCloudVisualizer.h"
#include "AutoRetouchInterface.h"
#include "VisualizationConfig.h"
#include <common/ConfigInterface.h>

using namespace hiveObliquePhotography::Visualization;

CInteractionCallback::CInteractionCallback(pcl::visualization::PCLVisualizer* vVisualizer)
{
	m_pVisualizer = CPointCloudVisualizer::getInstance();

	_ASSERTE(vVisualizer);
	vVisualizer->registerKeyboardCallback([&](const auto& vEvent) { keyboardCallback(vEvent); });
	vVisualizer->registerMouseCallback([&](const auto& vEvent) { mouseCallback(vEvent); });
	vVisualizer->registerAreaPickingCallback([&](const auto& vEvent) { areaPicking(vEvent); });

	if (hiveConfig::hiveParseConfig("VisualizationConfig.xml", hiveConfig::EConfigType::XML, CVisualizationConfig::getInstance()) != hiveConfig::EParseResult::SUCCEED)
		std::cout << "Failed to parse config file." << std::endl;
	else
		m_pVisualizationConfig = CVisualizationConfig::getInstance();

}

//*****************************************************************
//FUNCTION: 
void CInteractionCallback::keyboardCallback(const pcl::visualization::KeyboardEvent& vEvent)
{
	unsigned char KeyAscii = vEvent.getKeyCode();
	std::string KeyString = vEvent.getKeySym();

	if (KeyAscii != 0 && KeyAscii < 256)
	{
		m_KeyPressStatus[vEvent.getKeyCode()] = vEvent.keyDown() ? true : false;
	}

	if (vEvent.keyDown())
	{
		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(VIEW_BINARY_RESULT).value())
		{
			AutoRetouch::hiveExecuteBinaryClassifier(AutoRetouch::CLASSIFIER_BINARY_VFH);
			m_pVisualizer->refresh();
		}

		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(SWITCH_BINARY_GROWING).value())
		{
			m_PartitionMode = !m_PartitionMode;
		}

		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(SWITCH_BINARY_CLUSTER_LABEL).value())
		{
			m_UnwantedMode = !m_UnwantedMode;
		}

		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(SWITCH_UNWANTED_DISCARD).value())
		{
			static int i = 0;
			i++;
			if (i % 2)
				AutoRetouch::hiveSwitchPointLabel(AutoRetouch::EPointLabel::DISCARDED, AutoRetouch::EPointLabel::UNWANTED);
			else
				AutoRetouch::hiveSwitchPointLabel(AutoRetouch::EPointLabel::UNWANTED, AutoRetouch::EPointLabel::DISCARDED);
			m_pVisualizer->refresh();
		}
	}

}

//*****************************************************************
//FUNCTION: 
void CInteractionCallback::mouseCallback(const pcl::visualization::MouseEvent& vEvent)
{
	auto Button = vEvent.getButton();
	bool PressStatus = (vEvent.getType() == pcl::visualization::MouseEvent::MouseButtonPress) ? true : false;

	if (Button == pcl::visualization::MouseEvent::LeftButton)
		m_MousePressStatus[0] = PressStatus;
	else if (Button == pcl::visualization::MouseEvent::RightButton)
		m_MousePressStatus[1] = PressStatus;
}

//*****************************************************************
//FUNCTION: 
void CInteractionCallback::areaPicking(const pcl::visualization::AreaPickingEvent& vEvent)
{
	pcl::Indices Indices;
	vEvent.getPointsIndices(Indices);

	pcl::visualization::Camera Camera;
	m_pVisualizer->m_pPCLVisualizer->getCameraParameters(Camera);

	if (m_PartitionMode)
		AutoRetouch::hiveExecuteClusterAlg2CreateCluster(Indices, m_UnwantedMode ? AutoRetouch::EPointLabel::UNWANTED : AutoRetouch::EPointLabel::KEPT, Camera);
	else
		AutoRetouch::hiveExecuteClusterAlg2RegionGrowing(Indices, m_UnwantedMode ? AutoRetouch::EPointLabel::UNWANTED : AutoRetouch::EPointLabel::KEPT, Camera);

	m_pVisualizer->refresh();
}
