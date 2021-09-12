#include "pch.h"
#include "PointCloudVisualizer.h"
#include "PointCloudRetouchInterface.h"

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
void CPointCloudVisualizer::refresh(const std::vector<std::size_t>& vPointLabel, bool vResetCamera)
{
	RECORD_TIME_BEGIN

	_ASSERTE(!m_pSceneCloud->empty());

	m_pPCLVisualizer->removeAllPointClouds();
	m_pPCLVisualizer->removeAllShapes();
	
	_ASSERTE(vPointLabel.size() == m_pSceneCloud->size());

	auto PointSize = *CVisualizationConfig::getInstance()->getAttribute<double>(POINT_SHOW_SIZE);
	if (m_VisualFlag & EVisualFlag::ShowCloud)
	{
		VisualCloud_t::Ptr pCloud2Show(new VisualCloud_t);
		pCloud2Show->resize(m_pSceneCloud->size());
		std::memcpy(pCloud2Show->data(), m_pSceneCloud->data(), m_pSceneCloud->size() * sizeof(VisualPoint_t));

		for (int i = 0; i < m_pSceneCloud->size(); i++)
		{
			switch (vPointLabel[i])
			{
			case 0:
				pCloud2Show->points[i].a = 0;
				break;
			case 1:
			{
				unsigned char KeptHighlightColor[4] = { std::get<2>(m_BackgroundColor), std::get<1>(m_BackgroundColor), std::get<0>(m_BackgroundColor), 255 };
				std::memcpy(&pCloud2Show->points[i].rgba, KeptHighlightColor, sizeof(KeptHighlightColor));
				break;
			}
			case 2:
			{
				unsigned char UnwantedHighlightColor[4] = { std::get<2>(m_LitterColor), std::get<1>(m_LitterColor), std::get<0>(m_LitterColor), 255 };	//gbr
				std::memcpy(&pCloud2Show->points[i].rgba, UnwantedHighlightColor, sizeof(UnwantedHighlightColor));
				break;
			}
			case 3:
				pCloud2Show->points[i].a = 255;
				break;
			case 4:
			{
				unsigned char StandardWhite[4] = { 255, 255, 255, 255 };
				std::memcpy(&pCloud2Show->points[i].rgba, StandardWhite, sizeof(StandardWhite));
				break;
			}
			}
		}

		//show user defined color
		{
			for (auto& Record : m_UserColoredPoints)
				if (!Record.IsNewCloud)
					for (auto Index : Record.PointSet)
						if (Index < m_pSceneCloud->size())
						{
							unsigned char UserColor[4] = { Record.Color.z(), Record.Color.y(), Record.Color.x(), 255 };
							std::memcpy(&pCloud2Show->points[Index].rgba, UserColor, sizeof(UserColor));
						}
		}

		pcl::visualization::PointCloudColorHandlerRGBAField<VisualPoint_t> RGBAColor(pCloud2Show);
		m_pPCLVisualizer->addPointCloud<VisualPoint_t>(pCloud2Show, RGBAColor, "Cloud2Show");
		m_pPCLVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PointSize, "Cloud2Show");

		for (int i = 0; i < m_UserColoredPoints.size(); i++)
		{
			auto& Record = m_UserColoredPoints[i];
			if (Record.IsNewCloud)
			{
				VisualCloud_t::Ptr pUserPoints(new VisualCloud_t);

				for (auto Index : Record.PointSet)
				{
					VisualPoint_t TempPoint = m_pSceneCloud->points[Index];
					TempPoint.x += Record.DeltaPos.x();
					TempPoint.y += Record.DeltaPos.y();
					TempPoint.z += Record.DeltaPos.z();
					TempPoint.r = Record.Color.x();
					TempPoint.g = Record.Color.y();
					TempPoint.b = Record.Color.z();
					pUserPoints->push_back(TempPoint);
				}

				m_pPCLVisualizer->addPointCloud<VisualPoint_t>(pUserPoints, "UserPoints" + std::to_string(i));
				m_pPCLVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, Record.PointSize, "UserPoints" + std::to_string(i));
			}
		}
	}

	if (m_VisualFlag & EVisualFlag::ShowUserCloud)
	{
		for (int i = 0; i < m_UserCloudSet.size(); i++)
		{
			VisualCloud_t::Ptr TempCloud(new VisualCloud_t);
			pcl::copyPointCloud(*m_UserCloudSet[i], *TempCloud);
			m_pPCLVisualizer->addPointCloud<VisualPoint_t>(TempCloud, "UserCloud" + std::to_string(i));
			m_pPCLVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PointSize, "UserCloud" + std::to_string(i));
		}
	}

	if (m_VisualFlag & EVisualFlag::ShowMesh)
	{
		for (auto& Pair : m_MeshSet)
		{
			const std::string MeshPrefix = "Mesh";
			static int i = 0;
			if (Pair.first != "")
				m_pPCLVisualizer->removePolygonMesh(Pair.first);
			else
			{
				Pair.first = MeshPrefix + std::to_string(i++);
				m_pPCLVisualizer->addTextureMesh(Pair.second, Pair.first);
			}
		}
	}

	if (vResetCamera)
	{
		m_pPCLVisualizer->resetCamera();
		pcl::visualization::Camera Camera;
		m_pPCLVisualizer->getCameraParameters(Camera);
		m_WindowSize = { Camera.window_size[0], Camera.window_size[1] };
	}

	m_pPCLVisualizer->updateCamera();

	RECORD_TIME_END(显示)
}

//*****************************************************************
//FUNCTION: 
void CPointCloudVisualizer::run()
{
	while (!m_pPCLVisualizer->wasStopped())
	{
		m_pPCLVisualizer->spinOnce(16);
	}
}

//*****************************************************************
//FUNCTION: 
int CPointCloudVisualizer::addUserColoredPoints(const std::vector<pcl::index_t>& vPointSet, const Eigen::Vector3i& vColor)
{
	static int HighlightId = -1;
	HighlightId++;
	m_UserColoredPoints.push_back({ vPointSet, vColor, { 0.0f, 0.0f, 0.0f }, CVisualizationConfig::getInstance()->getAttribute<double>(POINT_SHOW_SIZE).value(), false, HighlightId });
	return HighlightId;
}

//*****************************************************************
//FUNCTION: 
int CPointCloudVisualizer::addUserColoredPointsAsNewCloud(const std::vector<pcl::index_t>& vPointSet, const Eigen::Vector3i& vColor, const Eigen::Vector3f& vDeltaPos, double vPointSize)
{
	static int HighlightId = -1;
	HighlightId++;
	m_UserColoredPoints.push_back({ vPointSet, vColor, vDeltaPos, vPointSize, true, HighlightId });
	return HighlightId;
}

//*****************************************************************
//FUNCTION: 
void CPointCloudVisualizer::removeUserColoredPoints(int vId)
{
	for (auto Iter = m_UserColoredPoints.begin(); Iter != m_UserColoredPoints.end(); Iter++)
	{
		if (Iter->Id == vId)
		{
			m_UserColoredPoints.erase(Iter);
			return;
		}
	}
}

//*****************************************************************
//FUNCTION: 
void CPointCloudVisualizer::jumpToThreeView(EView vViewType)
{
	Eigen::Vector3f MinCoord = m_AabbBox.first;
	Eigen::Vector3f MaxCoord = m_AabbBox.second;
	auto Offset = MaxCoord - MinCoord;
	int MinDis = INT_MAX; int MaxDis = -INT_MAX;
	int MinDirection = -1; int MaxDirection = -1; int MidDirection = -1;
	for(int i = 0;i < 3;i++)
	{
		if(Offset[i] < MinDis)
		{
			MinDis = Offset[i];
			MinDirection = i;
		}
		if(Offset[i] > MaxDis)
		{
			MaxDis = Offset[i];
			MaxDirection = i;
		}
	}
	for (int i = 0; i < 3; i++)
		if (i != MinDirection && i != MaxDirection)
			MidDirection = i;
	float MaxRate = Offset[MaxDirection] / Offset[MinDirection];
	float MinRate = Offset[MaxDirection] / Offset[MidDirection];
	Eigen::Vector3f CenterPos = (MinCoord + MaxCoord) / 2.0f;
	Eigen::Vector3f TopViewPos;
	Eigen::Vector3f MianViewPos;
	Eigen::Vector3f SideViewPos;
	Eigen::Vector3f Up;
	Eigen::Vector3f Front;
	for(int k = 0; k < 3;k++)
	{
		if(k == MinDirection)
		{
			TopViewPos[k] = CenterPos[k] + 2 * MaxRate * Offset[k];
			MianViewPos[k] = CenterPos[k];
			SideViewPos[k] = CenterPos[k];
			Up[k] = 1.0;
			Front[k] = 0.0;
		}
		else if(k == MaxDirection)
		{
			TopViewPos[k] = CenterPos[k];
			MianViewPos[k] = CenterPos[k];
			SideViewPos[k] = CenterPos[k] - 2  * Offset[k];
			Up[k] = 0.0;
			Front[k] = 0.0;
		}
		else
		{
			TopViewPos[k] = CenterPos[k];
			MianViewPos[k] = CenterPos[k] + 2 * MinRate * Offset[k];
			SideViewPos[k] = CenterPos[k];
			Up[k] = 0.0;
			Front[k] = -1.0;
		}
	}
 
	pcl::visualization::Camera Camera;
	m_pPCLVisualizer->getCameraParameters(Camera);
	Camera.focal[0] = CenterPos.x(); Camera.focal[1] = CenterPos.y(); Camera.focal[2] = CenterPos.z();
	Camera.view[0] = Up.x(); Camera.view[1] = Up.y(); Camera.view[2] = Up.z();
	switch (vViewType)
	{
	    case EView::TopView:
			Camera.pos[0] = TopViewPos.x(); Camera.pos[1] = TopViewPos.y(); Camera.pos[2] = TopViewPos.z();
			Camera.view[0] = Front.x(); Camera.view[1] = Front.y(); Camera.view[2] = Front.z();
			break;
		case EView::MainView:
			Camera.pos[0] = MianViewPos.x(); Camera.pos[1] = MianViewPos.y(); Camera.pos[2] = MianViewPos.z();
			Camera.view[0] = Up.x(); Camera.view[1] = Up.y(); Camera.view[2] = Up.z();
			break;
		case EView::SideView:
			Camera.pos[0] = SideViewPos.x(); Camera.pos[1] = SideViewPos.y(); Camera.pos[2] = SideViewPos.z();
			Camera.view[0] = Up.x(); Camera.view[1] = Up.y(); Camera.view[2] = Up.z();
			break;
	}
	Camera.clip[0] = 0.1;
	Camera.clip[1] = 10000.0;
	m_pPCLVisualizer->setCameraParameters(Camera);
	m_pPCLVisualizer->updateCamera();
}

//*****************************************************************
//FUNCTION: 
void CPointCloudVisualizer::showBoundingBox()
{
	Eigen::Vector3f MinCoord = m_AabbBox.first;
	Eigen::Vector3f MaxCoord = m_AabbBox.second;
	
	Eigen::Vector3f Vertex1{ MaxCoord.x(), MinCoord.y(), MinCoord.z() };  pcl::PointXYZ Point1(Vertex1.x(), Vertex1.y(), Vertex1.z());
	Eigen::Vector3f Vertex2{ MaxCoord.x(), MaxCoord.y(), MinCoord.z() };  pcl::PointXYZ Point2(Vertex2.x(), Vertex2.y(), Vertex2.z());
	Eigen::Vector3f Vertex3{ MinCoord.x(), MaxCoord.y(), MinCoord.z() };  pcl::PointXYZ Point3(Vertex3.x(), Vertex3.y(), Vertex3.z());
	Eigen::Vector3f Vertex4{ MinCoord.x(), MaxCoord.y(), MaxCoord.z() };  pcl::PointXYZ Point4(Vertex4.x(), Vertex4.y(), Vertex4.z());
	Eigen::Vector3f Vertex5{ MinCoord.x(), MinCoord.y(), MaxCoord.z() };  pcl::PointXYZ Point5(Vertex5.x(), Vertex5.y(), Vertex5.z());
	Eigen::Vector3f Vertex6{ MaxCoord.x(), MinCoord.y(), MaxCoord.z() };  pcl::PointXYZ Point6(Vertex6.x(), Vertex6.y(), Vertex6.z());
	pcl::PointXYZ Point0(MinCoord.x(), MinCoord.y(), MinCoord.z());
	pcl::PointXYZ Point7(MaxCoord.x(), MaxCoord.y(), MaxCoord.z());
	m_pPCLVisualizer->addLine(Point0, Point1, 1.0, 1.0, 1.0, "0"); m_pPCLVisualizer->addLine(Point0, Point3, 1.0, 1.0, 1.0, "1");
	m_pPCLVisualizer->addLine(Point0, Point5, 1.0, 1.0, 1.0, "2"); m_pPCLVisualizer->addLine(Point1, Point2, 1.0, 1.0, 1.0, "3");
	m_pPCLVisualizer->addLine(Point2, Point3, 1.0, 1.0, 1.0, "4"); m_pPCLVisualizer->addLine(Point3, Point4, 1.0, 1.0, 1.0, "5");
	m_pPCLVisualizer->addLine(Point4, Point5, 1.0, 1.0, 1.0, "6"); m_pPCLVisualizer->addLine(Point5, Point6, 1.0, 1.0, 1.0, "7");
	m_pPCLVisualizer->addLine(Point6, Point1, 1.0, 1.0, 1.0, "8"); m_pPCLVisualizer->addLine(Point7, Point2, 1.0, 1.0, 1.0, "9");
	m_pPCLVisualizer->addLine(Point7, Point4, 1.0, 1.0, 1.0, "10"); m_pPCLVisualizer->addLine(Point7, Point6, 1.0, 1.0, 1.0, "11");
}