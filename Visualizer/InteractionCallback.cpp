#include "pch.h"
#include "InteractionCallback.h"
#include "PointCloudVisualizer.h"
#include "VisualizationConfig.h"
#include "PointCloudRetouchInterface.h"
#include <common/ConfigInterface.h>
#include <omp.h>
#include <mutex>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>
#include <pcl/sample_consensus/impl/ransac.hpp>
#include "PlanarityFeature.h"

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

		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(REMOVE_OUTLIER).value())
		{
			PointCloudRetouch::hiveMarkIsolatedAreaAsLitter();
			std::vector<std::size_t> PointLabel;
			PointCloudRetouch::hiveDumpPointLabel(PointLabel);
			m_pVisualizer->refresh(PointLabel);
		}

		if (vEvent.isCtrlPressed() && KeyString == m_pVisualizationConfig->getAttribute<std::string>(UNDO).value())
		{
			PointCloudRetouch::hiveUndo();
			std::vector<std::size_t> PointLabel;
			PointCloudRetouch::hiveDumpPointLabel(PointLabel);
			m_pVisualizer->refresh(PointLabel);
		}

		if (KeyString == "t")
		{
			pcl::visualization::Camera Camera;
			m_pVisualizer->m_pPCLVisualizer->getCameraParameters(Camera);
			m_pVisualizer->m_pPCLVisualizer->saveCameraParameters("TestCameraInfo.txt");
		}

		if (KeyString == "Delete")
		{
			hiveObliquePhotography::PointCloudRetouch::hiveClearMark();
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
	bool ScrollStatus = (vEvent.getType() == pcl::visualization::MouseEvent::MouseScrollDown || vEvent.getType() == pcl::visualization::MouseEvent::MouseScrollUp) ? true : false;

	if (Button == pcl::visualization::MouseEvent::LeftButton)
	{
		m_MousePressStatus[0] = PressStatus;
	}
	else if (Button == pcl::visualization::MouseEvent::RightButton)
	{
		m_MousePressStatus[1] = PressStatus;
	}
	else if (Button == pcl::visualization::MouseEvent::MiddleButton)
	{
		m_MousePressStatus[2] = PressStatus;
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
			std::optional<double> ScreenCircleRadius = m_pVisualizationConfig->getAttribute<double>(SCREEN_CIRCLE_RADIUS);
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

	if (m_pVisualizationConfig->getAttribute<bool>(CIRCLE_MODE).value() && m_MousePressStatus[1])
	{
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->setLineMode(true);

		//reset
		m_pVisualizer->m_MainColors.clear();
		m_pVisualizer->removeAllUserColoredPoints();

		{
			std::vector<std::size_t> PointLabel;
			PointCloudRetouch::hiveDumpPointLabel(PointLabel);
			m_pVisualizer->refresh(PointLabel);
		}

		if (m_pVisualizationConfig)
		{
			std::optional<double> ScreenCircleRadius = m_pVisualizationConfig->getAttribute<double>(SCREEN_CIRCLE_RADIUS);
			if (ScreenCircleRadius.has_value())
				m_Radius = ScreenCircleRadius.value();
			std::optional<double> ScreenCircleHardness = m_pVisualizationConfig->getAttribute<double>(SCREEN_CIRCLE_HARDNESS);
			if (ScreenCircleHardness.has_value())
				m_Hardness = ScreenCircleHardness.value();

			std::optional<bool> UnwantedMode = m_pVisualizationConfig->getAttribute<bool>(UNWANTED_MODE);
			if (UnwantedMode.has_value())
				m_UnwantedMode = UnwantedMode.value();
			std::optional<bool> RefreshImmediately = m_pVisualizationConfig->getAttribute<bool>(REFRESH_IMMEDIATELY);
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

		m_Radius = static_cast<int>(m_Radius);
		double RadiusOnWindow = m_Radius * Camera.window_size[1] / m_pVisualizer->m_WindowSize.y();
		Eigen::Vector2d CircleCenterOnWindow = { PosX, PosY };

		std::vector<pcl::index_t> PickedIndices;
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->switchMode(true);
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->areaPick(PosX - RadiusOnWindow, PosY - RadiusOnWindow, PosX + RadiusOnWindow, PosY + RadiusOnWindow, PickedIndices);	//rectangle
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->switchMode(false);
		hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("Successfully pick %1% points.", PickedIndices.size()));

		if (PickedIndices.empty())
			return;

		auto DistanceFunc = [&](const Eigen::Vector2d& vPos) -> double
		{
			Eigen::Vector2d PosOnWindow((vPos.x() + 1) * Camera.window_size[0] / 2, (vPos.y() + 1) * Camera.window_size[1] / 2);

			if ((PosOnWindow - CircleCenterOnWindow).norm() <= RadiusOnWindow)
				return -1;
			else
				return 1;
		};

		PointCloudRetouch::hivePreprocessSelected(PickedIndices, PV, DistanceFunc, ViewPos);
		//m_pVisualizer->addUserColoredPoints(PickedIndices, { 255, 255, 255 });

		auto HardnessFunc = [=](const Eigen::Vector2d& vPos) -> double
		{
			Eigen::Vector2d PosOnWindow((vPos.x() + 1) * Camera.window_size[0] / 2, (vPos.y() + 1) * Camera.window_size[1] / 2);

			double X = (PosOnWindow - CircleCenterOnWindow).norm() / RadiusOnWindow;
			if (X <= 1.0)
			{
				X -= m_Hardness;
				if (X < 0)
					return 1.0;
				X /= (1 - m_Hardness);
				X *= X;

				return X * (X - 2) + 1;
			}
			else
				return 0;
		};
		if (m_UnwantedMode)
			PointCloudRetouch::hiveMarkLitter(PickedIndices, PV, HardnessFunc);
		else
			PointCloudRetouch::hiveMarkBackground(PickedIndices, PV, HardnessFunc);

		std::vector<pcl::index_t> NearestPoints;
		if (PointCloudRetouch::hiveDumpColorFeatureNearestPoints(NearestPoints))
		{
			for (auto Index : NearestPoints)
			{
				auto& Point = m_pVisualizer->m_pSceneCloud->points[Index];
				Eigen::Vector3i Color{ Point.r, Point.g, Point.b };
				m_pVisualizer->addUserColoredPointsAsNewCloud({ Index }, Color, 30.0);
			}
		}

		if (m_IsRefreshImmediately)
		{
			std::vector<std::size_t> PointLabel;
			PointCloudRetouch::hiveDumpPointLabel(PointLabel);
			m_pVisualizer->refresh(PointLabel);
		}

		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->setLineMode(false);
	}

	//draw point picking hint circle
	if (m_pVisualizationConfig->getAttribute<bool>(CIRCLE_MODE).value())
	{
		m_pVisualizer->m_pPCLVisualizer->removeAllShapes();

		//colors
		PointCloudRetouch::hiveDumpColorFeatureMainColors(m_pVisualizer->m_MainColors);

		pcl::visualization::Camera Camera;
		m_pVisualizer->m_pPCLVisualizer->getCameraParameters(Camera);

		Eigen::Matrix4d Proj, View;
		Camera.computeProjectionMatrix(Proj);
		Camera.computeViewMatrix(View);

		if (m_pVisualizer->m_MainColors.size())
		{
			pcl::visualization::Camera Camera;
			m_pVisualizer->m_pPCLVisualizer->getCameraParameters(Camera);

			float CircleY = 0.7, DistanceX = 0.2;
			std::vector<Eigen::Vector2f> CirclesNDCPos;

			float DeltaX = float(m_pVisualizer->m_MainColors.size() - 1) * 0.5f;

			for (int i = 0; i < m_pVisualizer->m_MainColors.size(); i++)
			{
				CirclesNDCPos.push_back({ (i - DeltaX) * DistanceX, CircleY });
			}

			for (int i = 0; i < m_pVisualizer->m_MainColors.size(); i++)
			{
				Eigen::Vector4d PixelPosition = { CirclesNDCPos[i].x(), CirclesNDCPos[i].y(), -0.5f, 1.0f };

				PixelPosition = (Proj * View).inverse() * PixelPosition;
				PixelPosition /= PixelPosition.w();

				pcl::PointXYZ Circle;

				Circle.x = PixelPosition.x();
				Circle.y = PixelPosition.y();
				Circle.z = PixelPosition.z();

				Eigen::Vector3d CameraPos{ Camera.pos[0], Camera.pos[1], Camera.pos[2] };
				Eigen::Vector3d PixelPos{ PixelPosition.x(), PixelPosition.y(), PixelPosition.z() };

				auto Length = (CameraPos - PixelPos).norm();

				std::string CircleName = "Circle" + std::to_string(i);
				if (!m_MousePressStatus[0] && !m_MousePressStatus[2])
				{
					m_pVisualizer->m_pPCLVisualizer->addSphere<pcl::PointXYZ>(Circle, 0.3 / m_pVisualizer->m_WindowSize.y() * Length * m_pVisualizationConfig->getAttribute<double>(SCREEN_CIRCLE_RADIUS).value(), float(m_pVisualizer->m_MainColors[i].x()) / 255, float(m_pVisualizer->m_MainColors[i].y()) / 255, float(m_pVisualizer->m_MainColors[i].z()) / 255, CircleName);
				}
			}
		}

		Eigen::Vector4d PixelPosition = { PosX / Camera.window_size[0] * 2 - 1, PosY / Camera.window_size[1] * 2 - 1, 0.0f, 1.0f };

		PixelPosition = (Proj * View).inverse() * PixelPosition;
		PixelPosition /= PixelPosition.w();

		pcl::PointXYZ Circle;

		Circle.x = PixelPosition.x();
		Circle.y = PixelPosition.y();
		Circle.z = PixelPosition.z();

		Eigen::Vector3d CameraPos{ Camera.pos[0], Camera.pos[1], Camera.pos[2] };
		Eigen::Vector3d PixelPos{ PixelPosition.x(), PixelPosition.y(), PixelPosition.z() };

		auto Length = (CameraPos - PixelPos).norm();

		if (!m_MousePressStatus[0] && !m_MousePressStatus[2])
		{
			m_pVisualizer->m_pPCLVisualizer->addSphere<pcl::PointXYZ>(Circle, 0.5555 / m_pVisualizer->m_WindowSize.y() * Length * m_pVisualizationConfig->getAttribute<double>(SCREEN_CIRCLE_RADIUS).value(), 255, 255, 0, "Circle");
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
