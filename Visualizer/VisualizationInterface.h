#pragma once
#include "VisualizationExport.h"
#include "VisualizationConfig.h"
#include "VisualizationCommon.h"
#include "Mesh.h"

namespace hiveObliquePhotography
{
	namespace Visualization
	{
		VISUALIZATION_DECLSPEC void hiveInitVisualizer(const std::vector<RetouchCloud_t::Ptr>& vTileSet, bool vIsInQt = true);

		VISUALIZATION_DECLSPEC void hiveResetVisualizer(const std::vector<RetouchCloud_t::Ptr>& vTileSet, bool vIsInQt = true);

		VISUALIZATION_DECLSPEC void hiveRefreshVisualizer(const std::vector<std::size_t>& vPointLabel, bool vResetCamera = false);

		VISUALIZATION_DECLSPEC void hiveSetPointRenderSize(double vSize);

		VISUALIZATION_DECLSPEC void hiveRunVisualizerLoop();

		VISUALIZATION_DECLSPEC void hiveSetVisualFlag(int vFlag);
		
		VISUALIZATION_DECLSPEC int hiveHighlightPointSet(const std::vector<pcl::index_t>& vPointSet, const Eigen::Vector3i& vColor);

		VISUALIZATION_DECLSPEC void hiveCancelHighlighting(int vId);

		VISUALIZATION_DECLSPEC void hiveCancelAllHighlighting();

		VISUALIZATION_DECLSPEC void hiveRemoveAllShapes();

		VISUALIZATION_DECLSPEC void hiveAddTextureMesh(const pcl::TextureMesh& vMesh);

		VISUALIZATION_DECLSPEC void hiveDumpUserCloudSet(std::vector<RetouchCloud_t::Ptr>& voCloudSet);

		VISUALIZATION_DECLSPEC pcl::visualization::PCLVisualizer*& hiveGetPCLVisualizer();

		VISUALIZATION_DECLSPEC bool hiveGetVisualizationConfig(CVisualizationConfig*& voConfig);

		VISUALIZATION_DECLSPEC void TestInterface(const CMesh& vMesh, const std::string& vIndicesPath);
	}
}