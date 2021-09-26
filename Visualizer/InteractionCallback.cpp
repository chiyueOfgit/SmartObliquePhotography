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
	m_pVisualizationConfig = CVisualizationConfig::getInstance();

	_ASSERTE(vVisualizer);
	vVisualizer->registerKeyboardCallback([&](const auto& vEvent) { keyboardCallback(vEvent); });
	vVisualizer->registerMouseCallback([&](const auto& vEvent) { mouseCallback(vEvent); });
	vVisualizer->registerPointPickingCallback([&](const auto& vEvent) { pointPicking(vEvent); });
	vVisualizer->registerAreaPickingCallback([&](const auto& vEvent) { areaPicking(vEvent); });
}

//*****************************************************************
//FUNCTION: 
void CInteractionCallback::keyboardCallback(const pcl::visualization::KeyboardEvent& vEvent)
{
	unsigned char KeyAscii = vEvent.getKeyCode();
	std::string KeyString = vEvent.getKeySym();
	bool RefreshFlag = false;

	if (KeyAscii != 0 && KeyAscii < 256)
		m_KeyPressStatus[KeyAscii] = vEvent.keyDown() ? true : false;

	if (vEvent.keyDown())
	{
		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(SWITCH_UNWANTED_KEPT_MODE).value())
			m_pVisualizationConfig->overwriteAttribute(UNWANTED_MODE, !m_pVisualizationConfig->getAttribute<bool>(UNWANTED_MODE).value());

		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(SWITCH_UNWANTED_DISCARD).value())
		{
			static int i = 0;
			if (++i % 2)
				PointCloudRetouch::hiveHideLitter();
			else
				PointCloudRetouch::hiveDisplayLitter();	
			RefreshFlag = true;
		}

		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(REMOVE_OUTLIER).value())
		{
			PointCloudRetouch::hiveMarkIsolatedAreaAsLitter();
			RefreshFlag = true;
		}

		if (vEvent.isCtrlPressed() && KeyString == m_pVisualizationConfig->getAttribute<std::string>(UNDO).value())
		{
			PointCloudRetouch::hiveUndo();
			RefreshFlag = true;
		}

		const double RadiusStep = 5.0;
		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(RADIUS_UP).value())
			m_pVisualizationConfig->overwriteAttribute(SCREEN_CIRCLE_RADIUS, m_pVisualizationConfig->getAttribute<double>(SCREEN_CIRCLE_RADIUS).value() + RadiusStep);
		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(RADIUS_DOWN).value())
			m_pVisualizationConfig->overwriteAttribute(SCREEN_CIRCLE_RADIUS, m_pVisualizationConfig->getAttribute<double>(SCREEN_CIRCLE_RADIUS).value() - RadiusStep);

		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(POINT_SIZE_UP).value())
		{
			m_pVisualizationConfig->overwriteAttribute(POINT_SHOW_SIZE, m_pVisualizationConfig->getAttribute<double>(POINT_SHOW_SIZE).value() + 1);
			m_pVisualizer->setPointRenderSize(m_pVisualizationConfig->getAttribute<double>(POINT_SHOW_SIZE).value());
		}

		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(POINT_SIZE_DOWN).value())
		{
			m_pVisualizationConfig->overwriteAttribute(POINT_SHOW_SIZE, m_pVisualizationConfig->getAttribute<double>(POINT_SHOW_SIZE).value() - 1);
			m_pVisualizer->setPointRenderSize(m_pVisualizationConfig->getAttribute<double>(POINT_SHOW_SIZE).value());
		}

		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(RECOVER_BACKGROUND_POINTS).value())
		{
			hiveObliquePhotography::PointCloudRetouch::hiveRecoverBackgroundMark();
			RefreshFlag = true;
		}

		if (KeyString == "r")
		{
			std::vector<RetouchPoint_t> NewPoints;
			PointCloudRetouch::hiveRepairHole(NewPoints);
			if (!NewPoints.empty())
			{
				RetouchCloud_t::Ptr RepairCloud(new RetouchCloud_t);
				for (auto& Point : NewPoints)
				{
					if (Point.r == 0 && Point.g == 0 && Point.b == 0)
						Point.rgba = -1;
					RepairCloud->push_back(Point);
				}

				m_pVisualizer->addUserPointCloud(RepairCloud);
				RefreshFlag = true;
			}
		}

		/*if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(JUMP_TO_MAIN_VIEW).value())
		{
			m_pVisualizer->jumpToThreeView(EView::MainView);
		}
		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(JUMP_TO_TOP_VIEW).value())
		{
			m_pVisualizer->jumpToThreeView(EView::TopView);
		}
		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(JUMP_TO_SIDE_VIEW).value())
		{
			m_pVisualizer->jumpToThreeView(EView::SideView);
		}
		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(SHOW_BOUNDING_BOX).value())
		{
			m_pVisualizer->showBoundingBox();
		}*/

		if (RefreshFlag)
		{
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

	m_pVisualizer->m_pPCLVisualizer->getCameraParameters(m_Camera);
	Eigen::Matrix4d Proj, View;
	m_Camera.computeProjectionMatrix(Proj);
	m_Camera.computeViewMatrix(View);
	Eigen::Matrix4d PV = Proj * View;
	Eigen::Vector3d ViewPos = { m_Camera.pos[0], m_Camera.pos[1], m_Camera.pos[2] };

	static bool Scroll;
	if (m_pVisualizationConfig->getAttribute<bool>(AUTO_LOD).value() && ((Button == pcl::visualization::MouseEvent::LeftButton && vEvent.getType() == pcl::visualization::MouseEvent::MouseButtonRelease) || (Scroll && vEvent.getType() == pcl::visualization::MouseEvent::MouseMove)))
		m_pVisualizer->__autoLod();
	Scroll = ScrollStatus;

	if (m_pVisualizationConfig->getAttribute<bool>(CIRCLE_MODE).value() && Button == pcl::visualization::MouseEvent::RightButton && PressStatus)
	{
		m_pVisualizer->m_pPCLVisualizer->removeAllShapes();
		m_pVisualizer->m_pPCLVisualizer->updateCamera();

		m_Radius = m_pVisualizationConfig->getAttribute<double>(SCREEN_CIRCLE_RADIUS).value();
		m_Hardness = m_pVisualizationConfig->getAttribute<double>(SCREEN_CIRCLE_HARDNESS).value();
		m_UnwantedMode = m_pVisualizationConfig->getAttribute<bool>(UNWANTED_MODE).value();
		m_IsRefreshImmediately = m_pVisualizationConfig->getAttribute<bool>(REFRESH_IMMEDIATELY).value();

		m_Radius = static_cast<int>(m_Radius);
		double RadiusOnWindow = m_Radius * m_Camera.window_size[1] / m_pVisualizer->m_WindowSize.y();
		Eigen::Vector2d CircleCenterOnWindow = { m_PosX, m_PosY };

		std::vector<pcl::index_t> PickedIndices;
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->switchMode(true);
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->areaPick(m_PosX - RadiusOnWindow, m_PosY - RadiusOnWindow, m_PosX + RadiusOnWindow, m_PosY + RadiusOnWindow, PickedIndices);	//rectangle
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->switchMode(false);

		if (PickedIndices.empty())
			return;

		Eigen::Vector3f PickWorldPos{};
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->singlePick(m_PosX, m_PosY, PickWorldPos.x(), PickWorldPos.y(), PickWorldPos.z());
		int WhichTile = 0;
		for (; WhichTile < m_pVisualizer->m_TileBoxSet.size(); WhichTile++)
			if (isInAABB(PickWorldPos, m_pVisualizer->m_TileBoxSet[WhichTile]))
				break;
		_ASSERTE(WhichTile < m_pVisualizer->m_TileSet.size());
		for (auto& Index : PickedIndices)
			Index += m_pVisualizer->m_OffsetSet[WhichTile];

		auto DistanceFunc = [&](const Eigen::Vector2d& vPos) -> double
		{
			Eigen::Vector2d PosOnWindow((vPos.x() + 1) * m_Camera.window_size[0] / 2, (vPos.y() + 1) * m_Camera.window_size[1] / 2);

			if ((PosOnWindow - CircleCenterOnWindow).norm() <= RadiusOnWindow)
				return -1;
			else
				return 1;
		};

		PointCloudRetouch::hivePreprocessSelected(PickedIndices, PV, DistanceFunc, ViewPos);
		m_pVisualizer->addUserColoredPoints(PickedIndices, { 255, 255, 255 });

		auto HardnessFunc = [=](const Eigen::Vector2d& vPos) -> double
		{
			Eigen::Vector2d PosOnWindow((vPos.x() + 1) * m_Camera.window_size[0] / 2, (vPos.y() + 1) * m_Camera.window_size[1] / 2);

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

		if (m_IsRefreshImmediately)
		{
			std::vector<std::size_t> PointLabel;
			PointCloudRetouch::hiveDumpTileLabel(WhichTile, PointLabel);
			m_pVisualizer->refresh(WhichTile, PointLabel);
		}
	}

	else if (m_pVisualizationConfig->getAttribute<bool>(AREA_MODE).value())
	{
		m_pVisualizer->m_pPCLVisualizer->removeAllShapes();

		m_UnwantedMode = m_pVisualizationConfig->getAttribute<bool>(UNWANTED_MODE).value();

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

			if (!m_pVisualizationConfig->getAttribute<bool>(REPAIR_MODE).value())
			{
				Eigen::Vector3f PickWorldPos{};
				m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->singlePick(m_PosX, m_PosY, PickWorldPos.x(), PickWorldPos.y(), PickWorldPos.z());
				int WhichTile = 0;
				for (; WhichTile < m_pVisualizer->m_TileBoxSet.size(); WhichTile++)
					if (isInAABB(PickWorldPos, m_pVisualizer->m_TileBoxSet[WhichTile]))
						break;
				_ASSERTE(WhichTile < m_pVisualizer->m_TileSet.size());
				for (auto& Index : PickedIndices)
					Index += m_pVisualizer->m_OffsetSet[WhichTile];

				PointCloudRetouch::hiveTagLabel(PickedIndices, m_UnwantedMode);
				std::vector<std::size_t> PointLabel;
				PointCloudRetouch::hiveDumpTileLabel(WhichTile, PointLabel);
				m_pVisualizer->refresh(WhichTile, PointLabel);
			}
			else
			{
				if (m_UnwantedMode)
					PointCloudRetouch::hiveRepairHoleSetRepairRegion(PickedIndices);
				else
					PointCloudRetouch::hiveRepairHoleSetReferenceRegion(PickedIndices);
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
				Eigen::Vector4d PixelPosition = { vCoord.x() / m_Camera.window_size[0] * 2 - 1, vCoord.y() / m_Camera.window_size[1] * 2 - 1, -0.8f, 1.0f };
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

			Eigen::Vector3i SquareColor = m_UnwantedMode ? m_pVisualizer->m_LitterColor : m_pVisualizer->m_BackgroundColor;
			for (int i = 0; i < LineEndPoints.size(); i++)
			{
				auto& LineStartPoint = LineEndPoints[i];
				auto& LineEndPoint = LineEndPoints[(i + 1) % LineEndPoints.size()];
				m_pVisualizer->m_pPCLVisualizer->addLine(LineStartPoint, LineEndPoint, SquareColor.x(), SquareColor.y(), SquareColor.z(), "Line" + std::to_string(i));
				m_pVisualizer->m_pPCLVisualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "Line" + std::to_string(i));
			}

			m_pVisualizer->m_pPCLVisualizer->updateCamera();
		}
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
	m_pVisualizer->m_pPCLVisualizer->removeAllShapes();
	m_UnwantedMode = m_pVisualizationConfig->getAttribute<bool>(UNWANTED_MODE).value();

	pcl::visualization::Camera Camera;
	m_pVisualizer->m_pPCLVisualizer->getCameraParameters(Camera);
	Eigen::Matrix4d Proj, View;
	Camera.computeProjectionMatrix(Proj);
	Camera.computeViewMatrix(View);
	Eigen::Matrix4d PVInverse = (Proj * View).inverse();

	Eigen::Vector4d PixelPosition = { m_PosX / Camera.window_size[0] * 2 - 1, m_PosY / Camera.window_size[1] * 2 - 1, 0.0f, 1.0f };
	PixelPosition = PVInverse * PixelPosition;
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
		Eigen::Vector3i CircleColor = m_UnwantedMode ? m_pVisualizer->m_LitterColor : m_pVisualizer->m_BackgroundColor;

		m_pVisualizer->m_pPCLVisualizer->addSphere<pcl::PointXYZ>(Circle, 0.5555 / m_pVisualizer->m_WindowSize.y() * Length * m_pVisualizationConfig->getAttribute<double>(SCREEN_CIRCLE_RADIUS).value(), CircleColor.x(), CircleColor.y(), CircleColor.z(), "Circle");
		m_pVisualizer->m_pPCLVisualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "Circle");
		m_pVisualizer->m_pPCLVisualizer->updateCamera();
	}
}