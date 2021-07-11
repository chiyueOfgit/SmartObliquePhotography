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

		//else if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(SWITCH_BINARY_GROWING).value())
		//{
		//	m_PartitionMode = !m_PartitionMode;
		//}

		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(SWITCH_BINARY_CLUSTER_LABEL).value())
		{
			m_UnwantedMode = !m_UnwantedMode;
		}

		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(CLUSTER_EXPANDER_MODE).value())
		{
			static bool bCircleMode = m_pVisualizationConfig->getAttribute<bool>("CIRCLE_MODE").value();
			bCircleMode = !bCircleMode;
			m_pVisualizationConfig->overwriteAttribute("CIRCLE_MODE", bCircleMode);
		}

		//else if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(SWITCH_LINEPICK).value())
		//{
		//	m_LineMode = !m_LineMode;
		//	m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->setLineMode(m_LineMode);
		//}

		//else if (KeyString == "m")
		//{
		//	pcl::Indices InputIndice;
		//	AutoRetouch::hiveGetIndicesByLabel(InputIndice, AutoRetouch::EPointLabel::UNDETERMINED);
		//	AutoRetouch::hiveExecuteOutlierDetecting(InputIndice, AutoRetouch::EPointLabel::UNWANTED);
		//	m_pVisualizer->refresh();
		//}

		//else if (vEvent.isCtrlPressed() && KeyString == m_pVisualizationConfig->getAttribute<std::string>(UNDO).value())
		//{
		//	AutoRetouch::hiveUndoLastOp();
		//	m_pVisualizer->refresh();
		//}

		//else if (KeyString == "s")
		//{
		//	AutoRetouch::hiveExecuteCompositeBinaryClassifier();
		//	m_pVisualizer->refresh();
		//}
		//
		if (KeyString == m_pVisualizationConfig->getAttribute<std::string>(SWITCH_UNWANTED_DISCARD).value())
		{
			static int i = 0;
			i++;
			if (i % 2)
				PointCloudRetouch::hiveDiscardUnwantedPoints();
			else
				PointCloudRetouch::hiveRecoverDiscardPoints2Unwanted();

			std::vector<std::size_t> PointLabel;
			PointCloudRetouch::hiveDumpPointLabel(PointLabel);
			m_pVisualizer->refresh(PointLabel);
		}

		if (KeyString == "t")
		{
			//m_pVisualizer->m_pPCLVisualizer->saveCameraParameters("Camera");
			PointCloudRetouch::hiveRemoveOutlier();
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

	if (m_pVisualizationConfig->getAttribute<bool>("RUBBER_MODE").value())
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->setLineMode(true);
	else
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->setLineMode(false);

	if (m_pVisualizationConfig->getAttribute<bool>("RUBBER_MODE").value() && m_MousePressStatus[0])
	{
		{
			std::vector<std::size_t> PointLabel;
			PointCloudRetouch::hiveDumpPointLabel(PointLabel);
			m_pVisualizer->refresh(PointLabel);
		}

		if (m_pVisualizationConfig)
		{
			std::optional<float> ScreenCircleRadius = m_pVisualizationConfig->getAttribute<double>("SCREEN_CIRCLE_RADIUS");
			if (ScreenCircleRadius.has_value())
				m_Radius = ScreenCircleRadius.value();
		}

		std::vector<pcl::index_t> PickedIndices;
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->switchMode(true);
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->areaPick(PosX - m_Radius, PosY - m_Radius, PosX + m_Radius, PosY + m_Radius, PickedIndices);
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->switchMode(false);

		PointCloudRetouch::hiveExecuteRubber(PickedIndices);

		{
			std::vector<std::size_t> PointLabel;
			PointCloudRetouch::hiveDumpPointLabel(PointLabel);
			m_pVisualizer->refresh(PointLabel);
		}
	}

	if (m_pVisualizationConfig->getAttribute<bool>("CIRCLE_MODE").value() && m_MousePressStatus[1] && OnceMousePressStatus[1])
	{
		{
			std::vector<std::size_t> PointLabel;
			PointCloudRetouch::hiveDumpPointLabel(PointLabel);
			m_pVisualizer->refresh(PointLabel);
		}

		if (m_pVisualizationConfig)
		{
			std::optional<float> ScreenCircleRadius = m_pVisualizationConfig->getAttribute<double>("SCREEN_CIRCLE_RADIUS");
			if (ScreenCircleRadius.has_value())
				m_Radius = ScreenCircleRadius.value();

			std::optional<float> ScreenCircleHardness = m_pVisualizationConfig->getAttribute<double>("SCREEN_CIRCLE_HARDNESS");
			if (ScreenCircleHardness.has_value())
				m_Hardness = ScreenCircleHardness.value();
		}

		m_Radius = (int)m_Radius;

		std::vector<pcl::index_t> PickedIndices;
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->switchMode(true);
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->areaPick(PosX - m_Radius, PosY - m_Radius, PosX + m_Radius, PosY + m_Radius, PickedIndices);	//rectangle
		m_pVisualizer->m_pPCLVisualizer->getInteractorStyle()->switchMode(false);
		hiveEventLogger::hiveOutputEvent(_FORMAT_STR1("Successfully pick %1% points.", PickedIndices.size()));

		pcl::visualization::Camera Camera;
		m_pVisualizer->m_pPCLVisualizer->getCameraParameters(Camera);

		// test ray
		int RectangleLength = 2 * m_Radius + 1;
		std::vector<float> PointsDepth(RectangleLength * RectangleLength, FLT_MAX);
		std::set<int> Points;

		Eigen::Matrix4d Proj, View;
		Camera.computeProjectionMatrix(Proj);
		Camera.computeViewMatrix(View);
		Eigen::Matrix4f PV = (Proj * View).cast<float>();
		Eigen::Matrix4f PVInverse = PV.inverse();

		auto Cloud = *m_pVisualizer->m_pSceneCloud;

		int BeginX = PosX - m_Radius;
		int BeginY = PosY - m_Radius;

		//tile projection
		const std::size_t NumPartitionX = 0.5 * m_Radius, NumPartitionY = 0.5 * m_Radius;
		float TileDeltaX = (float)RectangleLength / NumPartitionX;
		float TileDeltaY = (float)RectangleLength / NumPartitionY;

		std::vector<std::vector<pcl::index_t>> PointTile(NumPartitionX * NumPartitionY);

		std::mutex Mutex;

#pragma omp parallel for
		for (int i = 0; i < PickedIndices.size(); i++ )
		{
			auto Index = PickedIndices[i];
			auto& Point = Cloud[Index];
			Eigen::Vector4f Position{ Point.x, Point.y, Point.z, 1.0f };

			Position = PV * Position;
			Position /= Position.eval().w();
			Position += Eigen::Vector4f(1.0, 1.0, 1.0, 1.0);
			Position /= 2.0;
			Position.x() *= Camera.window_size[0];
			Position.y() *= Camera.window_size[1];
			Eigen::Vector3f Coord{ Position.x(), Position.y(), Position.z() };

			int TileIndexX = (Position.x() - BeginX) / TileDeltaX;
			int TileIndexY = (Position.y() - BeginY) / TileDeltaY;

			_ASSERTE(TileIndexX >= 0 && TileIndexY >= 0);

			Mutex.lock();
			PointTile[TileIndexX + TileIndexY * NumPartitionX].push_back(Index);
			Mutex.unlock();
		}

#pragma omp parallel for
		for (int Y = BeginY; Y <= int(PosY + m_Radius); Y++)
		{
#pragma omp parallel for
			for (int X = BeginX; X <= int(PosX + m_Radius); X++)
			{
				std::map<float, int> DepthAndIndices;

				Eigen::Vector4f PixelPosition = { float(X / Camera.window_size[0] * 2 - 1), float(Y / Camera.window_size[1] * 2 - 1), 0.0f, 1.0f };

				PixelPosition = PVInverse * PixelPosition;
				PixelPosition /= PixelPosition.w();

				Eigen::Vector3f RayOrigin{ (float)Camera.pos[0], (float)Camera.pos[1], (float)Camera.pos[2] };
				Eigen::Vector3f RayDirection = {PixelPosition.x() - RayOrigin.x(), PixelPosition.y() - RayOrigin.y(), PixelPosition.z() - RayOrigin.z() };
				RayDirection /= RayDirection.norm();

				int TileIndexX = (X - BeginX) / TileDeltaX;
				int TileIndexY = (Y - BeginY) / TileDeltaY;

				auto& Tile = PointTile[TileIndexX + TileIndexY * NumPartitionX];

				for (auto Index : Tile)
				{
					auto& Point = m_pVisualizer->m_pSceneCloud->points[Index];
					Eigen::Vector3f Pos{ Point.x, Point.y, Point.z };
					Eigen::Vector3f Normal{ Point.normal_x, Point.normal_y, Point.normal_z };

					float K = (Pos - RayOrigin).dot(Normal) / RayDirection.dot(Normal);

					Eigen::Vector3f IntersectPosition = RayOrigin + K * RayDirection;

					const float SurfelRadius = 1.0f;	//surfel world radius

					if ((IntersectPosition - Pos).norm() < SurfelRadius)
							DepthAndIndices[K] = Index;
				}

				int Offset = X - BeginX + (Y - BeginY) * RectangleLength;
				_ASSERTE(Offset >= 0);

				const float WorldLengthLimit = 0.5f;	//magic
				if (Offset < PointsDepth.size() && !DepthAndIndices.empty())
				{
					auto MinDepth = DepthAndIndices.begin()->first;
					for (auto& Pair : DepthAndIndices)
					{
						if (Pair.first - MinDepth < WorldLengthLimit)
						{
							Mutex.lock();
							Points.insert(Pair.second);
							Mutex.unlock();
						}

					}
				}
			}
		}

		m_pVisualizer->addUserColoredPoints({Points.begin(), Points.end()}, { 255, 255, 255 });

		if (m_UnwantedMode)                                                                                   
			PointCloudRetouch::hiveMarkLitter(PickedIndices, m_Hardness, m_Radius, { PosX, PosY }, Proj * View, { Camera.window_size[0], Camera.window_size[1] });
		else
			PointCloudRetouch::hiveMarkBackground(PickedIndices, m_Hardness, m_Radius, { PosX, PosY }, Proj * View, { Camera.window_size[0], Camera.window_size[1] });

		m_IsRefreshImmediately = m_pVisualizationConfig->getAttribute<bool>(REFRESH_IMMEDIATELY).value();
		if (m_IsRefreshImmediately)
		{
			std::vector<std::size_t> PointLabel;
			PointCloudRetouch::hiveDumpPointLabel(PointLabel);
			m_pVisualizer->refresh(PointLabel);
		}

	}

	if (m_pVisualizationConfig->getAttribute<bool>("CIRCLE_MODE").value())
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
			m_pVisualizer->m_pPCLVisualizer->addSphere<pcl::PointXYZ>(Circle, 0.001 * Length * m_pVisualizationConfig->getAttribute<double>("SCREEN_CIRCLE_RADIUS").value(), 255, 255, 0, "Circle");
			m_pVisualizer->m_pPCLVisualizer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "Circle");
			m_pVisualizer->m_pPCLVisualizer->updateCamera();
		}
	}

}

//*****************************************************************
//FUNCTION: 
void hiveObliquePhotography::Visualization::CInteractionCallback::pointPicking(const pcl::visualization::PointPickingEvent& vEvent)
{

}

//*****************************************************************
//FUNCTION: 
void CInteractionCallback::areaPicking(const pcl::visualization::AreaPickingEvent& vEvent)
{

}
