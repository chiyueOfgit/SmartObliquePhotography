#include "pch.h"
#include "VisualizationInterface.h"
#include "PointCloudVisualizer.h"

#include <fstream>	//remove
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/set.hpp>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>
#include "Mesh.h"
#include "PointCloudRetouchInterface.h"

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
	CPointCloudVisualizer::getInstance()->m_MainColors.clear();
	CPointCloudVisualizer::getInstance()->m_pPCLVisualizer->removeAllShapes();
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

void hiveObliquePhotography::Visualization::TestInterface(const std::string& vObj, const std::string& vBoundary)
{
	auto pVisualizer = CPointCloudVisualizer::getInstance();

	std::set<int> BoundaryPoints;
	std::ifstream file(vBoundary);
	boost::archive::text_iarchive ia(file);
	ia >> BOOST_SERIALIZATION_NVP(BoundaryPoints);
	file.close();

	pcl::TextureMesh Mesh1, Mesh2;
	pcl::io::loadOBJFile(vObj, Mesh1);
	pcl::io::loadPolygonFileOBJ(vObj, Mesh2);
	Mesh2.tex_materials = Mesh1.tex_materials;
	auto Material = Mesh2.tex_materials[0];

	CMesh M(Mesh1);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	for (auto Boundary : BoundaryPoints)
	{
		pcl::PointXYZRGB Point;
		auto& Vertex = M.m_Vertices[Boundary];
		Point.x = Vertex.x;
		Point.y = Vertex.y;
		Point.z = Vertex.z;
		Point.r = 255;
		Point.g = 0;
		Point.b = 0;
		pCloud->push_back(Point);
	}

	auto pPCLVisualizer = hiveGetPCLVisualizer();
	pPCLVisualizer->addTextureMesh(Mesh2);
	pPCLVisualizer->addPointCloud(pCloud, vBoundary);
	pPCLVisualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, vBoundary);
	pPCLVisualizer->resetCamera();
	pPCLVisualizer->updateCamera();
}
