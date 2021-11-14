#include "pch.h"
#include "VisualizationInterface.h"
#include "PointCloudVisualizer.h"

#include <fstream>	//remove
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/vector.hpp>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>
#include "PointCloudRetouchInterface.h"
#include "ObliquePhotographyDataInterface.h"

using namespace hiveObliquePhotography::Visualization;

void hiveObliquePhotography::Visualization::hiveInitVisualizer(const std::vector<RetouchCloud_t::Ptr>& vTileSet, bool vIsInQt)
{
	CPointCloudVisualizer::getInstance()->init(vTileSet, vIsInQt);
}

void hiveObliquePhotography::Visualization::hiveResetVisualizer(const std::vector<RetouchCloud_t::Ptr>& vTileSet, bool vIsInQt)
{
	CPointCloudVisualizer::getInstance()->reset(vTileSet, vIsInQt);
}

void hiveObliquePhotography::Visualization::hiveRefreshVisualizer(const std::vector<std::size_t>& vPointLabel, bool vResetCamera)
{
	CPointCloudVisualizer::getInstance()->refresh(vPointLabel, vResetCamera);
}

void hiveObliquePhotography::Visualization::hiveSetPointRenderSize(double vSize)
{
	CPointCloudVisualizer::getInstance()->setPointRenderSize(vSize);
}

void hiveObliquePhotography::Visualization::hiveRunVisualizerLoop()
{
	CPointCloudVisualizer::getInstance()->run();
}

void hiveObliquePhotography::Visualization::hiveSetVisualFlag(int vFlag)
{
	CPointCloudVisualizer::getInstance()->setVisualFlag(vFlag);
}

int hiveObliquePhotography::Visualization::hiveHighlightPointSet(const std::vector<pcl::index_t>& vPointSet, const Eigen::Vector3i& vColor)
{
	if (!vPointSet.empty())
	{
		auto MaxIter = std::max_element(vPointSet.begin(), vPointSet.end());
		_ASSERTE(MaxIter != vPointSet.end() && *MaxIter < CPointCloudVisualizer::getInstance()->m_NumPoints);
	}
	_ASSERTE(vColor.x() >= 0 && vColor.y() >= 0 && vColor.z() >= 0);

	return CPointCloudVisualizer::getInstance()->addUserColoredPoints(vPointSet, vColor);
}

void hiveObliquePhotography::Visualization::hiveCancelHighlighting(int vId)
{
	CPointCloudVisualizer::getInstance()->removeUserColoredPoints(vId);
}

void hiveObliquePhotography::Visualization::hiveCancelAllHighlighting()
{
	CPointCloudVisualizer::getInstance()->removeAllUserColoredPoints();
}

void hiveObliquePhotography::Visualization::hiveRemoveAllShapes()
{
	hiveGetPCLVisualizer()->removeAllShapes();
}

void hiveObliquePhotography::Visualization::hiveAddTextureMesh(const pcl::TextureMesh& vMesh)
{
	CPointCloudVisualizer::getInstance()->addTextureMesh(vMesh);
}

void hiveObliquePhotography::Visualization::hiveDumpUserCloudSet(std::vector<RetouchCloud_t::Ptr>& voCloudSet)
{
	voCloudSet = CPointCloudVisualizer::getInstance()->getUserCloudSet();
}

pcl::visualization::PCLVisualizer*& hiveObliquePhotography::Visualization::hiveGetPCLVisualizer()
{
	return CPointCloudVisualizer::getInstance()->m_pPCLVisualizer;
}

bool hiveObliquePhotography::Visualization::hiveGetVisualizationConfig(CVisualizationConfig*& voConfig)
{
	auto pConfig = CVisualizationConfig::getInstance();
	_ASSERTE(pConfig);
	if (pConfig)
	{
		voConfig = CVisualizationConfig::getInstance();
		return true;
	}
	else
	{
		voConfig = nullptr;
		return false;
	}
}

void hiveObliquePhotography::Visualization::TestInterface(const CMesh& vMesh, const std::string& vIndicesPath)
{
	auto pVisualizer = CPointCloudVisualizer::getInstance();

	std::vector<int> IndciesPoints;
	std::ifstream file(vIndicesPath);
	boost::archive::text_iarchive ia(file);
	ia >> BOOST_SERIALIZATION_NVP(IndciesPoints);
	file.close();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	float Color = 0, Step = (float)255 / IndciesPoints.size();
	for (auto Boundary : IndciesPoints)
	{
		pcl::PointXYZRGB Point;
		auto& Vertex = vMesh.m_Vertices[Boundary];
		Point.x = Vertex.x;
		Point.y = Vertex.y;
		Point.z = Vertex.z;
		if (Color == 0)
		{
			Point.r = 0;
			Point.g = 255;
			Point.b = 0;
		}
		else
		{
			Point.r = 255 - Color;
			Point.g = 0;
			Point.b = Color;
		}
		pCloud->push_back(Point);
		Color += Step;
	}

	auto pPCLVisualizer = hiveGetPCLVisualizer();
	pPCLVisualizer->addPointCloud(pCloud, vIndicesPath);
	pPCLVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, vIndicesPath);
	pPCLVisualizer->updateCamera();
}

void hiveObliquePhotography::Visualization::hiveChangeCameraLookStraightAxis(size_t vAxis)
{
	auto pPCLVisualizer = hiveGetPCLVisualizer();
	switch (vAxis)
	{
	case 1:
		pPCLVisualizer->setCameraPosition(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
		break;
	case 2:
		pPCLVisualizer->setCameraPosition(0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
		break;
	case 3:
		pPCLVisualizer->setCameraPosition(0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
		break;
	case 4:
		pPCLVisualizer->setCameraPosition(0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0);
		break;
	case 5:
		pPCLVisualizer->setCameraPosition(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0);
		break;
	case 6:
		pPCLVisualizer->setCameraPosition(0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);
		break;
	default:
		break;
	}

	pPCLVisualizer->resetCamera();
	pPCLVisualizer->updateCamera();
}