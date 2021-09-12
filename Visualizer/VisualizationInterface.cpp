#include "pch.h"
#include "VisualizationInterface.h"
#include "PointCloudVisualizer.h"

using namespace hiveObliquePhotography::Visualization;

void hiveObliquePhotography::Visualization::hiveInitVisualizer(RetouchCloud_t::Ptr vPointCloud, bool vIsInQt)
{
	CPointCloudVisualizer::getInstance()->init<RetouchCloud_t>(vPointCloud, vIsInQt);
}

void hiveObliquePhotography::Visualization::hiveResetVisualizer(RetouchCloud_t::Ptr vPointCloud, bool vIsInQt)
{
	CPointCloudVisualizer::getInstance()->reset<RetouchCloud_t>(vPointCloud, vIsInQt);
}

void hiveObliquePhotography::Visualization::hiveRefreshVisualizer(const std::vector<std::size_t>& vPointLabel, bool vResetCamera)
{
	CPointCloudVisualizer::getInstance()->refresh(vPointLabel, vResetCamera);
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
		_ASSERTE(MaxIter != vPointSet.end() && *MaxIter < CPointCloudVisualizer::getInstance()->m_pSceneCloud->size());
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