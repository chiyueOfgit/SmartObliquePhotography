#include "pch.h"
#include "InteractionCallback.h"
#include "PointCloudVisualizer.h"
#include "VisualizationConfig.h"
#include "PointCloudRetouchInterface.h"
#include <common/ConfigInterface.h>

#include <fstream>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>

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
		m_KeyPressStatus[vEvent.getKeyCode()] = vEvent.keyDown() ? true : false;

	if (vEvent.keyDown())
	{
		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(SWITCH_UNWANTED_KEPT_MODE).value())
		{
			m_pVisualizationConfig->overwriteAttribute(UNWANTED_MODE, !m_pVisualizationConfig->getAttribute<bool>(UNWANTED_MODE).value());
		}

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

		const double RadiusStep = 5.0;
		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(RADIUS_UP).value())
			m_pVisualizationConfig->overwriteAttribute(SCREEN_CIRCLE_RADIUS, m_pVisualizationConfig->getAttribute<double>(SCREEN_CIRCLE_RADIUS).value() + RadiusStep);
		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(RADIUS_DOWN).value())
			m_pVisualizationConfig->overwriteAttribute(SCREEN_CIRCLE_RADIUS, m_pVisualizationConfig->getAttribute<double>(SCREEN_CIRCLE_RADIUS).value() - RadiusStep);

		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(POINT_SIZE_UP).value())
		{
			m_pVisualizationConfig->overwriteAttribute(POINT_SHOW_SIZE, m_pVisualizationConfig->getAttribute<double>(POINT_SHOW_SIZE).value() + 1);
			std::vector<std::size_t> PointLabel;
			PointCloudRetouch::hiveDumpPointLabel(PointLabel);
			m_pVisualizer->refresh(PointLabel);
		}

		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(POINT_SIZE_DOWN).value())
		{
			m_pVisualizationConfig->overwriteAttribute(POINT_SHOW_SIZE, m_pVisualizationConfig->getAttribute<double>(POINT_SHOW_SIZE).value() - 1);
			std::vector<std::size_t> PointLabel;
			PointCloudRetouch::hiveDumpPointLabel(PointLabel);
			m_pVisualizer->refresh(PointLabel);
		}

		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(RECOVER_BACKGROUND_POINTS).value())
		{
			hiveObliquePhotography::PointCloudRetouch::hiveRecoverBackgroundMark();
			std::vector<std::size_t> PointLabel;
			PointCloudRetouch::hiveDumpPointLabel(PointLabel);
			m_pVisualizer->refresh(PointLabel);
		}

		//draw point picking hint circle
		if (m_pVisualizationConfig->getAttribute<bool>(CIRCLE_MODE).value())
			__drawHintCircle();
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
		m_MousePressStatus[0] = PressStatus;
	else if (Button == pcl::visualization::MouseEvent::RightButton)
		m_MousePressStatus[1] = PressStatus;
	else if (Button == pcl::visualization::MouseEvent::MiddleButton)
		m_MousePressStatus[2] = PressStatus;

	int DeltaX, DeltaY;
	DeltaX = vEvent.getX() - m_PosX;
	DeltaY = vEvent.getY() - m_PosY;
	m_PosX = vEvent.getX();
	m_PosY = vEvent.getY();

	pcl::visualization::Camera Camera;
	m_pVisualizer->m_pPCLVisualizer->getCameraParameters(Camera);
	Eigen::Matrix4d Proj, View;
	Camera.computeProjectionMatrix(Proj);
	Camera.computeViewMatrix(View);
	Eigen::Matrix4d PV = Proj * View;
	Eigen::Vector3d ViewPos = { Camera.pos[0], Camera.pos[1], Camera.pos[2] };

	m_pVisualizer->m_pPCLVisualizer->removeAllShapes();

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

		pcl::visualization::Camera Camera;
		m_pVisualizer->m_pPCLVisualizer->getCameraParameters(Camera);
		Eigen::Matrix4d Proj, View;
		Camera.computeProjectionMatrix(Proj);
		Camera.computeViewMatrix(View);
		Eigen::Matrix4d PV = Proj * View;
		Eigen::Vector3d ViewPos = { Camera.pos[0], Camera.pos[1], Camera.pos[2] };

		m_Radius = static_cast<int>(m_Radius);
		double RadiusOnWindow = m_Radius * Camera.window_size[1] / m_pVisualizer->m_WindowSize.y();
		Eigen::Vector2d CircleCenterOnWindow = { m_PosX, m_PosY };

		std::vector<pcl::index_t> PickedIndices;
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->switchMode(true);
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->areaPick(m_PosX - m_Radius, m_PosY - m_Radius, m_PosX + m_Radius, m_PosY + m_Radius, PickedIndices);
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->switchMode(false);

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

		PointCloudRetouch::hiveEraseMark(PickedIndices);

		{
			std::vector<std::size_t> PointLabel;
			PointCloudRetouch::hiveDumpPointLabel(PointLabel);
			m_pVisualizer->refresh(PointLabel);
		}
	}

	if (m_pVisualizationConfig->getAttribute<bool>(CIRCLE_MODE).value() && Button == pcl::visualization::MouseEvent::RightButton && PressStatus)
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
		Eigen::Vector2d CircleCenterOnWindow = { m_PosX, m_PosY };

		std::vector<pcl::index_t> PickedIndices;
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->switchMode(true);
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->areaPick(m_PosX - RadiusOnWindow, m_PosY - RadiusOnWindow, m_PosX + RadiusOnWindow, m_PosY + RadiusOnWindow, PickedIndices);	//rectangle
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->switchMode(false);

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

		//__saveIndices("PickedIndices.txt", PickedIndices);
		//m_pVisualizer->m_pPCLVisualizer->saveCameraParameters("Camera.txt");
		//std::vector<double> RadiusAndCircleCenter = { m_Hardness, RadiusOnWindow, CircleCenterOnWindow[0], CircleCenterOnWindow[1] };
		//std::ofstream file("WindowInfo.txt");
		//boost::archive::text_oarchive oa(file);
		//oa& BOOST_SERIALIZATION_NVP(RadiusAndCircleCenter);
		//file.close();

		if (m_UnwantedMode)
			PointCloudRetouch::hiveMarkLitter(PickedIndices, PV, HardnessFunc);
		else
			PointCloudRetouch::hiveMarkBackground(PickedIndices, PV, HardnessFunc);

		//show points
		std::vector<pcl::index_t> NearestPoints;
		if (m_IsEnableColorInfos && PointCloudRetouch::hiveDumpColorFeatureNearestPoints(NearestPoints))
		{
			for (auto Index : NearestPoints)
			{
				auto& Point = m_pVisualizer->m_pSceneCloud->points[Index];
				Eigen::Vector3f CameraPos2Point = { float(Camera.pos[0] - Point.x), float(Camera.pos[1] - Point.y), float(Camera.pos[2] - Point.z) };
				Eigen::Vector3i Color{ Point.r, Point.g, Point.b };
				m_pVisualizer->addUserColoredPointsAsNewCloud({ Index }, Color, 0.01f * CameraPos2Point, 30.0);
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

	else if (m_pVisualizationConfig->getAttribute<bool>(AREA_MODE).value())
	{
		static bool isPicking = false;
		static Eigen::Vector2i LeftUp;

		if (isPicking && Button == pcl::visualization::MouseEvent::RightButton && !PressStatus)
		{
			isPicking = false;

			std::vector<pcl::index_t> PickedIndices;
			m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->switchMode(true);
			m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->areaPick(LeftUp.x(), LeftUp.y(), m_PosX, m_PosY, PickedIndices);	//rectangle
			m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->switchMode(false);

			if (PickedIndices.empty())
				return;

			if (m_pVisualizationConfig->getAttribute<bool>(AREA_PICK_CULLING).value())
				PointCloudRetouch::hivePreprocessSelected(PickedIndices, PV, [&](const Eigen::Vector2d&) -> double {return -1; }, ViewPos);
			//m_pVisualizer->addUserColoredPoints(PickedIndices, { 255, 255, 255 });

			auto pRandomHardness = [=](const Eigen::Vector2d&) -> double
			{
				static int i = 0;
				return i++ % 2 ? 1.0 : 0.0;
			};

			if (m_UnwantedMode)
				PointCloudRetouch::hiveMarkLitter(PickedIndices, PV, pRandomHardness);
			else
				PointCloudRetouch::hiveMarkBackground(PickedIndices, PV, pRandomHardness);

			{
				std::vector<std::size_t> PointLabel;
				PointCloudRetouch::hiveDumpPointLabel(PointLabel);
				m_pVisualizer->refresh(PointLabel);
			}
		}
		if (Button == pcl::visualization::MouseEvent::RightButton && PressStatus)
		{
			isPicking = true;
			LeftUp = { m_PosX, m_PosY };
		}

		if (isPicking)
		{
			std::vector<pcl::PointXYZ> LineEndPoints;

			auto fromWindow2World = [&](const Eigen::Vector2i& vCoord) -> pcl::PointXYZ
			{
				Eigen::Vector4d PixelPosition = { vCoord.x() / Camera.window_size[0] * 2 - 1, vCoord.y() / Camera.window_size[1] * 2 - 1, 0.0f, 1.0f };
				PixelPosition = (Proj * View).inverse() * PixelPosition;
				PixelPosition /= PixelPosition.w();

				pcl::PointXYZ TempPoint;
				TempPoint.x = PixelPosition.x();
				TempPoint.y = PixelPosition.y();
				TempPoint.z = PixelPosition.z();
				return TempPoint;
			};

			LineEndPoints.push_back(fromWindow2World(LeftUp));
			LineEndPoints.push_back(fromWindow2World({ m_PosX, LeftUp.y() }));
			LineEndPoints.push_back(fromWindow2World({ m_PosX, m_PosY }));
			LineEndPoints.push_back(fromWindow2World({ LeftUp.x(), m_PosY }));

			for (int i = 0; i < LineEndPoints.size(); i++)
			{
				auto& LineStartPoint = LineEndPoints[i];
				auto& LineEndPoint = LineEndPoints[(i + 1) % LineEndPoints.size()];
				m_pVisualizer->m_pPCLVisualizer->addLine(LineStartPoint, LineEndPoint, "Line" + std::to_string(i));
				m_pVisualizer->m_pPCLVisualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "Line" + std::to_string(i));
			}

			m_pVisualizer->m_pPCLVisualizer->updateCamera();
		}
	}

	auto OptionLitterColor = m_pVisualizationConfig->getAttribute<std::tuple<int, int, int>>(LITTER_HIGHLIGHT_COLOR);
	auto OptionBackgroundColor = m_pVisualizationConfig->getAttribute<std::tuple<int, int, int>>(BACKGROUND_HIGHLIGHT_COLOR);
	if (OptionLitterColor.has_value() && OptionBackgroundColor.has_value())
	{
		m_pVisualizer->m_LitterColor = OptionLitterColor.value();
		m_pVisualizer->m_BackgroundColor = OptionBackgroundColor.value();
	}

	//draw point picking hint circle
	if (m_pVisualizationConfig->getAttribute<bool>(CIRCLE_MODE).value())
		__drawHintCircle();
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

//*****************************************************************
//FUNCTION: 
void CInteractionCallback::__saveIndices(const std::string& vPath, const std::vector<int>& vIndices) const
{
	std::ofstream file(vPath.c_str());
	boost::archive::text_oarchive oa(file);
	oa& BOOST_SERIALIZATION_NVP(vIndices);
	file.close();
}

//*****************************************************************
//FUNCTION: 
void CInteractionCallback::__loadIndices(const std::string& vPath, std::vector<int>& voIndices) const
{
	std::ifstream file(vPath.c_str());
	boost::archive::text_iarchive ia(file);
	ia >> BOOST_SERIALIZATION_NVP(voIndices);
	file.close();
}

//*****************************************************************
//FUNCTION: 
void CInteractionCallback::__drawHintCircle()
{
	std::optional<bool> UnwantedMode = m_pVisualizationConfig->getAttribute<bool>(UNWANTED_MODE);
	if (UnwantedMode.has_value())
		m_UnwantedMode = UnwantedMode.value();  

	pcl::visualization::Camera Camera;
	m_pVisualizer->m_pPCLVisualizer->getCameraParameters(Camera);

	Eigen::Matrix4d Proj, View;
	Camera.computeProjectionMatrix(Proj);
	Camera.computeViewMatrix(View);

	//colors
	PointCloudRetouch::hiveDumpColorFeatureMainColors(m_pVisualizer->m_MainColors);

	if (m_pVisualizer->m_MainColors.size() && m_IsEnableColorInfos)
	{
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
				m_pVisualizer->m_pPCLVisualizer->addSphere<pcl::PointXYZ>(Circle, 0.3 / m_pVisualizer->m_WindowSize.y() * Length * 40.0, float(m_pVisualizer->m_MainColors[i].x()) / 255, float(m_pVisualizer->m_MainColors[i].y()) / 255, float(m_pVisualizer->m_MainColors[i].z()) / 255, CircleName);
			}
		}
	}

	Eigen::Vector4d PixelPosition = { m_PosX / Camera.window_size[0] * 2 - 1, m_PosY / Camera.window_size[1] * 2 - 1, 0.0f, 1.0f };

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
		Eigen::Vector3i CircleColor;
		if (m_UnwantedMode)
			CircleColor = { std::get<0>(m_pVisualizer->m_LitterColor), std::get<1>(m_pVisualizer->m_LitterColor), std::get<2>(m_pVisualizer->m_LitterColor) };
		else
			CircleColor = { std::get<0>(m_pVisualizer->m_BackgroundColor), std::get<1>(m_pVisualizer->m_BackgroundColor), std::get<2>(m_pVisualizer->m_BackgroundColor) };

		m_pVisualizer->m_pPCLVisualizer->addSphere<pcl::PointXYZ>(Circle, 0.5555 / m_pVisualizer->m_WindowSize.y() * Length * m_pVisualizationConfig->getAttribute<double>(SCREEN_CIRCLE_RADIUS).value(), CircleColor.x(), CircleColor.y(), CircleColor.z(), "Circle");
		m_pVisualizer->m_pPCLVisualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "Circle");
		m_pVisualizer->m_pPCLVisualizer->updateCamera();
	}
}
