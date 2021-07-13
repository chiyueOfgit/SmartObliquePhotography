#include "pch.h"
#include "InteractionCallback.h"
#include "PointCloudVisualizer.h"
#include "VisualizationConfig.h"
#include "PointCloudRetouchInterface.h"
#include <common/ConfigInterface.h>
#include <omp.h>
#include <mutex>

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
		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(SWITCH_UNWANTED_KEPT_MODE).value())
			m_pVisualizationConfig->overwriteAttribute(UNWANTED_MODE, !m_pVisualizationConfig->getAttribute<bool>(UNWANTED_MODE).value());

		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(CLUSTER_EXPANDER_MODE).value())
			m_pVisualizationConfig->overwriteAttribute(CIRCLE_MODE, !m_pVisualizationConfig->getAttribute<bool>(CIRCLE_MODE).value());

		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(SWITCH_UNWANTED_DISCARD).value())
		{
			static int i = 0;
			i++;
			if (i % 2)
				PointCloudRetouch::hiveHideLitter();
			else
				PointCloudRetouch::hiveDisplayLitter();

			std::vector<std::size_t> PointLabel;
			PointCloudRetouch::hiveDumpPointLabel(PointLabel);
			m_pVisualizer->refresh(PointLabel);
		}

		if (KeyString == REMOVE_OUTLIER)
		{
			PointCloudRetouch::hiveMarkIsolatedAreaAsLitter();
			std::vector<std::size_t> PointLabel;
			PointCloudRetouch::hiveDumpPointLabel(PointLabel);
			m_pVisualizer->refresh(PointLabel);
		}

		if (vEvent.isCtrlPressed() && KeyString == UNDO)
		{
			PointCloudRetouch::hiveUndo();
			std::vector<std::size_t> PointLabel;
			PointCloudRetouch::hiveDumpPointLabel(PointLabel);
			m_pVisualizer->refresh(PointLabel);
		}
	}
}

//*****************************************************************
//FUNCTION: 
void CInteractionCallback::mouseCallback(const pcl::visualization::MouseEvent& vEvent)
{
	auto Button = vEvent.getButton();
	bool PressStatus = (vEvent.getType() == pcl::visualization::MouseEvent::MouseButtonPress) ? true : false;
	bool OnceMousePressStatus[2] = { false };

	if (Button == pcl::visualization::MouseEvent::LeftButton)
	{
		m_MousePressStatus[0] = PressStatus;
		OnceMousePressStatus[0] = PressStatus;
	}
	else if (Button == pcl::visualization::MouseEvent::RightButton)
	{
		m_MousePressStatus[1] = PressStatus;
		OnceMousePressStatus[1] = PressStatus;
	}

	static int DeltaX, PosX, DeltaY, PosY;
	DeltaX = vEvent.getX() - PosX;
	DeltaY = vEvent.getY() - PosY;
	PosX = vEvent.getX();
	PosY = vEvent.getY();

	if (m_pVisualizationConfig->getAttribute<bool>(RUBBER_MODE).value())
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->setLineMode(true);
	else
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->setLineMode(false);

	if (m_pVisualizationConfig->getAttribute<bool>(RUBBER_MODE).value() && m_MousePressStatus[0])
	{
		{
			std::vector<std::size_t> PointLabel;
			PointCloudRetouch::hiveDumpPointLabel(PointLabel);
			m_pVisualizer->refresh(PointLabel);
		}

		if (m_pVisualizationConfig)
		{
			std::optional<float> ScreenCircleRadius = m_pVisualizationConfig->getAttribute<double>(SCREEN_CIRCLE_RADIUS);
			if (ScreenCircleRadius.has_value())
				m_Radius = ScreenCircleRadius.value();
		}

		std::vector<pcl::index_t> PickedIndices;
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->switchMode(true);
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->areaPick(PosX - m_Radius, PosY - m_Radius, PosX + m_Radius, PosY + m_Radius, PickedIndices);
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->switchMode(false);

		PointCloudRetouch::hiveEraseMark(PickedIndices);

		{
			std::vector<std::size_t> PointLabel;
			PointCloudRetouch::hiveDumpPointLabel(PointLabel);
			m_pVisualizer->refresh(PointLabel);
		}
	}

	if (m_pVisualizationConfig->getAttribute<bool>(CIRCLE_MODE).value() && m_MousePressStatus[1] && OnceMousePressStatus[1])
	{
		{
			std::vector<std::size_t> PointLabel;
			PointCloudRetouch::hiveDumpPointLabel(PointLabel);
			m_pVisualizer->refresh(PointLabel);
		}

		if (m_pVisualizationConfig)
		{
			std::optional<float> ScreenCircleRadius = m_pVisualizationConfig->getAttribute<double>(SCREEN_CIRCLE_RADIUS);
			if (ScreenCircleRadius.has_value())
				m_Radius = ScreenCircleRadius.value();
			std::optional<float> ScreenCircleHardness = m_pVisualizationConfig->getAttribute<double>(SCREEN_CIRCLE_HARDNESS);
			if (ScreenCircleHardness.has_value())
				m_Hardness = ScreenCircleHardness.value();

			std::optional<float> UnwantedMode = m_pVisualizationConfig->getAttribute<bool>(UNWANTED_MODE);
			if (UnwantedMode.has_value())
				m_UnwantedMode = UnwantedMode.value();
			std::optional<float> RefreshImmediately = m_pVisualizationConfig->getAttribute<bool>(REFRESH_IMMEDIATELY);
			if (RefreshImmediately.has_value())
				m_IsRefreshImmediately = RefreshImmediately.value();
		}

		pcl::visualization::Camera Camera;
		m_pVisualizer->m_pPCLVisualizer->getCameraParameters(Camera);
		Eigen::Matrix4d Proj, View;
		Camera.computeProjectionMatrix(Proj);
		Camera.computeViewMatrix(View);
		Eigen::Matrix4d PV = Proj * View;
		Eigen::Vector3d ViewPos = { Camera.pos[0], Camera.pos[1], Camera.pos[2] };

		m_Radius = (int)m_Radius;
		auto ZoomRate = Camera.window_size[1] / m_pVisualizer->m_WindowSize.y();
		auto ZoomedRadius = m_Radius * ZoomRate;

		std::vector<pcl::index_t> PickedIndices;
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->switchMode(true);
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->areaPick(PosX - ZoomedRadius, PosY - ZoomedRadius, PosX + ZoomedRadius, PosY + ZoomedRadius, PickedIndices);	//rectangle
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->switchMode(false);
		hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("Successfully pick %1% points.", PickedIndices.size()));

		Eigen::Vector2d CircleCenter = { 2 * (PosX / Camera.window_size[0]) - 1, 2 * (PosY / Camera.window_size[1]) - 1 };
		double RadiusInNDC = 2 * m_Radius / Camera.window_size[0] * ZoomRate;

		auto pFunc = [&](Eigen::Vector2d vPos) -> double
		{
			Eigen::Vector2d DeltaPos4Radius = { vPos.x() - CircleCenter.x(), (Camera.window_size[1] / Camera.window_size[0]) * (vPos.y() - CircleCenter.y()) };

			if (DeltaPos4Radius.norm() <= RadiusInNDC)
				return -1;
			else
				return 1;
		};

		PointCloudRetouch::hivePreprocessSelected(PickedIndices, PV, pFunc, ViewPos);
		m_pVisualizer->addUserColoredPoints(PickedIndices, { 255, 255, 255 });

		//if (m_UnwantedMode)                                                                                   
		//	PointCloudRetouch::hiveMarkLitter(PickedIndices, PV, m_Hardness);
		//else
		//	PointCloudRetouch::hiveMarkBackground(PickedIndices, PV, m_Hardness);

		if (m_IsRefreshImmediately)
		{
			std::vector<std::size_t> PointLabel;
			PointCloudRetouch::hiveDumpPointLabel(PointLabel);
			m_pVisualizer->refresh(PointLabel);
		}
	}

	//draw point picking hint circle
	if (m_pVisualizationConfig->getAttribute<bool>(CIRCLE_MODE).value())
	{
		pcl::visualization::Camera Camera;
		m_pVisualizer->m_pPCLVisualizer->getCameraParameters(Camera);

		Eigen::Vector4d PixelPosition = { PosX / Camera.window_size[0] * 2 - 1, PosY / Camera.window_size[1] * 2 - 1, 0.0f, 1.0f };

		Eigen::Matrix4d Proj, View;
		Camera.computeProjectionMatrix(Proj);
		Camera.computeViewMatrix(View);

		PixelPosition = (Proj * View).inverse() * PixelPosition;
		PixelPosition /= PixelPosition.w();

		pcl::PointXYZ Circle;

		Circle.x = PixelPosition.x();
		Circle.y = PixelPosition.y();
		Circle.z = PixelPosition.z();

		Eigen::Vector3d CameraPos{ Camera.pos[0], Camera.pos[1], Camera.pos[2] };
		Eigen::Vector3d PixelPos{ PixelPosition.x(), PixelPosition.y(), PixelPosition.z() };

		auto Length = (CameraPos - PixelPos).norm();

		m_pVisualizer->m_pPCLVisualizer->removeAllShapes();
		if (!m_MousePressStatus[0])
		{
			m_pVisualizer->m_pPCLVisualizer->addSphere<pcl::PointXYZ>(Circle, 0.00115 * Length * m_pVisualizationConfig->getAttribute<double>(SCREEN_CIRCLE_RADIUS).value(), 255, 255, 0, "Circle");
			m_pVisualizer->m_pPCLVisualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "Circle");
			m_pVisualizer->m_pPCLVisualizer->updateCamera();
		}
	}

}

//*****************************************************************
//FUNCTION: 
void CInteractionCallback::pointPicking(const pcl::visualization::PointPickingEvent& vEvent)
{

}

//*****************************************************************
//FUNCTION: 
void CInteractionCallback::areaPicking(const pcl::visualization::AreaPickingEvent& vEvent)
{

}
