#include "pch.h"
#include "PointCloudVisualizer.h"
#include "InteractionCallback.h"
#include "PointCloudRetouchInterface.h"
#include "VisualizationConfig.h"
#include <tuple>

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
void CPointCloudVisualizer::init(PointCloud_t::Ptr vPointCloud, bool vIsInQt)
{
	_ASSERTE(vPointCloud);
	m_pSceneCloud = vPointCloud;

	m_pPCLVisualizer = new pcl::visualization::PCLVisualizer("Visualizer", !vIsInQt);
	m_pCallback = new CInteractionCallback(m_pPCLVisualizer);
	m_pPCLVisualizer->setBackgroundColor(0.2, 0.2, 0.2);
	m_pPCLVisualizer->setShowFPS(false);
}

//*****************************************************************
//FUNCTION: 
void CPointCloudVisualizer::reset(PointCloud_t::Ptr vPointCloud, bool vIsInQt)
{
	m_pPCLVisualizer->removeAllPointClouds();
	delete m_pPCLVisualizer;
	delete m_pCallback;
	m_UserColoredPoints.clear();
	init(vPointCloud, vIsInQt);
	if (vPointCloud != nullptr)
		m_pSceneCloud = vPointCloud;
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

	PointCloud_t::Ptr pCloud2Show(new PointCloud_t);
	pCloud2Show->resize(m_pSceneCloud->size());
	std::memcpy(pCloud2Show->data(), m_pSceneCloud->data(), m_pSceneCloud->size() * sizeof(PointCloud_t::PointType));
	
	auto OptionLitterColor = CVisualizationConfig::getInstance()->getAttribute<std::tuple<int, int, int>>(LITTER_HIGHLIGHT_COLOR);
	auto OptionBackgroundColor = CVisualizationConfig::getInstance()->getAttribute<std::tuple<int, int, int>>(BACKGROUND_HIGHLIGHT_COLOR);
	if (OptionLitterColor.has_value() && OptionBackgroundColor.has_value())
	{
		m_LitterColor = OptionLitterColor.value();
		m_BackgroundColor = OptionBackgroundColor.value();
	}

	for (int i = 0; i < m_pSceneCloud->size(); i++)
	{
		switch (vPointLabel[i])
		{
		case 0:
		{
			pCloud2Show->points[i].a = 0;
			break;
		}
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
		{
			pCloud2Show->points[i].a = 255;
			break;
		}
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
		{
			if (!Record.IsNewCloud)
			{
				for (auto Index : Record.PointSet)
				{
					if (Index < m_pSceneCloud->size())
					{
						unsigned char UserColor[4] = { Record.Color.z(), Record.Color.y(), Record.Color.x(), 255 };
						std::memcpy(&pCloud2Show->points[Index].rgba, UserColor, sizeof(UserColor));
					}
				}
			}
		}
	}

	auto PointSize = *CVisualizationConfig::getInstance()->getAttribute<double>(POINT_SHOW_SIZE);
	
	pcl::visualization::PointCloudColorHandlerRGBAField<PointCloud_t::PointType> RGBAColor(pCloud2Show);
	m_pPCLVisualizer->addPointCloud(pCloud2Show, RGBAColor, "Cloud2Show");
	m_pPCLVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PointSize, "Cloud2Show");

	for (int i = 0; i < m_UserColoredPoints.size(); i++)
	{
		auto& Record = m_UserColoredPoints[i];
		if (Record.IsNewCloud)
		{
			PointCloud_t::Ptr pUserCloud(new PointCloud_t);

			for (auto Index : Record.PointSet)
			{
				pcl::PointSurfel TempPoint = m_pSceneCloud->points[Index];
				TempPoint.x += Record.DeltaPos.x();
				TempPoint.y += Record.DeltaPos.y();
				TempPoint.z += Record.DeltaPos.z();
				TempPoint.r = Record.Color.x();
				TempPoint.g = Record.Color.y();
				TempPoint.b = Record.Color.z();
				pUserCloud->push_back(TempPoint);
			}

			m_pPCLVisualizer->addPointCloud<pcl::PointSurfel>(pUserCloud, "UserCloud" + std::to_string(i));
			m_pPCLVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, Record.PointSize, "UserCloud" + std::to_string(i));
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