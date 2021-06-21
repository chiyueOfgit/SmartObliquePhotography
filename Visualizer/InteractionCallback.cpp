#include "pch.h"
#include "InteractionCallback.h"
#include "PointCloudVisualizer.h"
#include "VisualizationConfig.h"
#include "PointCloudRetouchScene.h"
#include <common/ConfigInterface.h>

using namespace hiveObliquePhotography::Visualization;

CInteractionCallback::CInteractionCallback(pcl::visualization::PCLVisualizer* vVisualizer)
{
	m_pVisualizer = CPointCloudVisualizer::getInstance();

	_ASSERTE(vVisualizer);
	vVisualizer->registerKeyboardCallback([&](const auto& vEvent) { keyboardCallback(vEvent); });
	vVisualizer->registerMouseCallback([&](const auto& vEvent) { mouseCallback(vEvent); });
	vVisualizer->registerPointPickingCallback([&](const auto& vEvent) { pointPicking(vEvent); });
	vVisualizer->registerAreaPickingCallback([&](const auto& vEvent) { areaPicking(vEvent); });

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
		//if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(VIEW_BINARY_RESULT).value())
		//{
		//	std::string ClusterType;
		//	if (m_pAutoRetouchConfig->getAttribute<bool>(KEY_WORDS::ENABLE_VFH).value())
		//		ClusterType = "vfh";
		//	else if (m_pAutoRetouchConfig->getAttribute<bool>(KEY_WORDS::ENABLE_SCORE).value())
		//		ClusterType = "score";
		//	else if (m_pAutoRetouchConfig->getAttribute<bool>(KEY_WORDS::ENABLE_NORMAL).value())
		//		ClusterType = "normal";
		//	AutoRetouch::hiveExecuteBinaryClassifier(AutoRetouch::CLASSIFIER_BINARY, ClusterType);
		//	m_pVisualizer->refresh();
		//}

		/*else if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(SWITCH_BINARY_GROWING).value())
		{
			m_PartitionMode = !m_PartitionMode;
		}

		else if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(SWITCH_BINARY_CLUSTER_LABEL).value())
		{
			m_UnwantedMode = !m_UnwantedMode;
		}

		else if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(SWITCH_LINEPICK).value())
		{
			m_LineMode = !m_LineMode;
			m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->setLineMode(m_LineMode);
		}

		else if (KeyString == "m")
		{
			pcl::Indices InputIndice;
			AutoRetouch::hiveGetIndicesByLabel(InputIndice, AutoRetouch::EPointLabel::UNDETERMINED);
			AutoRetouch::hiveExecuteOutlierDetecting(InputIndice, AutoRetouch::EPointLabel::UNWANTED);
			m_pVisualizer->refresh();
		}

		else if (vEvent.isCtrlPressed() && KeyString == m_pVisualizationConfig->getAttribute<std::string>(UNDO).value())
		{
			AutoRetouch::hiveUndoLastOp();
			m_pVisualizer->refresh();
		}

		else if (KeyString == "s")
		{
			AutoRetouch::hiveExecuteCompositeBinaryClassifier();
			m_pVisualizer->refresh();
		}
		
		else if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(SWITCH_UNWANTED_DISCARD).value())
		{
			static int i = 0;
			i++;
			if (i % 2)
				AutoRetouch::hiveSwitchPointLabel(AutoRetouch::EPointLabel::DISCARDED, AutoRetouch::EPointLabel::UNWANTED);
			else
				AutoRetouch::hiveSwitchPointLabel(AutoRetouch::EPointLabel::UNWANTED, AutoRetouch::EPointLabel::DISCARDED);
			m_pVisualizer->refresh();
		}*/
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

	static int DeltaX, PosX, DeltaY, PosY;
	DeltaX = vEvent.getX() - PosX;
	DeltaY = vEvent.getY() - PosY;
	PosX = vEvent.getX();
	PosY = vEvent.getY();
	
	if(m_LineMode)
	{
		if (m_MousePressStatus[0] || m_MousePressStatus[1])
		{
			std::vector<int> PickedIndices;
			pcl::visualization::Camera Camera;
			m_pVisualizer->m_pPCLVisualizer->getCameraParameters(Camera);
            const Eigen::Vector3f CameraPos{ static_cast<float>(Camera.pos[0]),static_cast<float>(Camera.pos[1]),static_cast<float>(Camera.pos[2]) };
			Eigen::Matrix4d ViewMatrix, ProjectionMatrix;
			Camera.computeViewMatrix(ViewMatrix);
			Camera.computeProjectionMatrix(ProjectionMatrix);
			const Eigen::Matrix4d PvMatrix = ProjectionMatrix * ViewMatrix;
			
			m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->linePick(PosX, PosY, PosX + DeltaX, PosY + DeltaY, m_pVisualizationConfig->getAttribute<float>(LINEWIDTH).value(), PickedIndices);
			pcl::IndicesPtr Indices = std::make_shared<pcl::Indices>(PickedIndices);
			//AutoRetouch::hiveExecuteMaxVisibilityClustering(Indices, m_UnwantedMode ? AutoRetouch::EPointLabel::UNWANTED : AutoRetouch::EPointLabel::UNDETERMINED, CameraPos, PvMatrix);

			//m_pVisualizer->refresh();
			
		}

		if (vEvent.getType() == pcl::visualization::MouseEvent::MouseButtonRelease
			&& vEvent.getButton() == pcl::visualization::MouseEvent::LeftButton
			&& m_KeyPressStatus['l'])
		{
			//»º´æ
			m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->switchMode(true);

			std::vector<int> PickedIndices;
			pcl::visualization::Camera Camera;
			m_pVisualizer->m_pPCLVisualizer->getCameraParameters(Camera);
			const Eigen::Vector3f CameraPos{ static_cast<float>(Camera.pos[0]),static_cast<float>(Camera.pos[1]),static_cast<float>(Camera.pos[2]) };
			Eigen::Matrix4d ViewMatrix, ProjectionMatrix;
			Camera.computeViewMatrix(ViewMatrix);
			Camera.computeProjectionMatrix(ProjectionMatrix);
			const Eigen::Matrix4d PvMatrix = ProjectionMatrix * ViewMatrix;

			//move to config
			m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->linePick(PosX, PosY, PosX + 10, PosY + 10, m_pVisualizationConfig->getAttribute<float>(LINEWIDTH).value(), PickedIndices);
			pcl::IndicesPtr Indices = std::make_shared<pcl::Indices>(PickedIndices);
			//AutoRetouch::hiveExecuteMaxVisibilityClustering(Indices, m_pVisualizationConfig->getAttribute<bool>(UNWANTED_MODE).value() ? AutoRetouch::EPointLabel::UNWANTED : AutoRetouch::EPointLabel::UNDETERMINED, CameraPos, PvMatrix);
			
			//m_pVisualizer->refresh();

			m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->switchMode(false);
		}
	}
}

//*****************************************************************
//FUNCTION: 
void hiveObliquePhotography::Visualization::CInteractionCallback::pointPicking(const pcl::visualization::PointPickingEvent& vEvent)
{
	//auto Index = vEvent.getPointIndex();
	//float GroundHeight = m_pVisualizer->m_pSceneCloud->points[Index].z + 0.1f;
	//	m_pAutoRetouchConfig->overwriteAttribute(KEY_WORDS::GROUND_TEST_THRESHOLD, GroundHeight);
}

//*****************************************************************
//FUNCTION: 
void CInteractionCallback::areaPicking(const pcl::visualization::AreaPickingEvent& vEvent)
{
	const pcl::IndicesPtr pIndices(new pcl::Indices);
	vEvent.getPointsIndices(*pIndices);

	pcl::visualization::Camera Camera;
	m_pVisualizer->m_pPCLVisualizer->getCameraParameters(Camera);
    const Eigen::Vector3f CameraPos = { static_cast<float>(Camera.pos[0]),static_cast<float>(Camera.pos[1]),static_cast<float>(Camera.pos[2]) };
	Eigen::Matrix4d ViewMatrix, ProjectionMatrix;
	Camera.computeViewMatrix(ViewMatrix);
	Camera.computeProjectionMatrix(ProjectionMatrix);
	const Eigen::Matrix4d PvMatrix = ProjectionMatrix * ViewMatrix;
	
	//if (m_PartitionMode)
	//	AutoRetouch::hiveExecuteClusterAlg2CreateCluster(pIndices, m_UnwantedMode ? AutoRetouch::EPointLabel::UNWANTED : AutoRetouch::EPointLabel::KEPT, CameraPos, PvMatrix);
	//else
	//	AutoRetouch::hiveExecuteCompositeClusterAndGrowing(pIndices, m_UnwantedMode ? AutoRetouch::EPointLabel::UNWANTED : AutoRetouch::EPointLabel::KEPT, CameraPos, PvMatrix);

	//m_pVisualizer->refresh();
}
