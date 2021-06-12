#include "pch.h"
#include "PointCluster4NormalRatio.h"
#include "PointCloudAutoRetouchScene.h"
#include <pcl/filters/impl/extract_indices.hpp>

using namespace hiveObliquePhotography::AutoRetouch;

CPointCluster4NormalRatio::CPointCluster4NormalRatio(const pcl::IndicesPtr& vPointIndices, EPointLabel vLabel) : IPointCluster(vLabel)
{	
	_ASSERTE(vPointIndices != nullptr);
	_ASSERTE(!vPointIndices->empty());

	const auto pCloud = CPointCloudAutoRetouchScene::getInstance()->getPointCloudScene();
	_ASSERTE(pCloud != nullptr);
	
	m_pPointCloud.reset(new PointCloud_t);
	pcl::ExtractIndices<PointCloud_t::PointType> Extract;
	Extract.setInputCloud(CPointCloudAutoRetouchScene::getInstance()->getPointCloudScene());
	Extract.setIndices(vPointIndices);
	Extract.filter(*m_pPointCloud);

	SBox Box;
	for (auto Index : *vPointIndices)
		Box.update(pCloud->at(Index).x, pCloud->at(Index).y, pCloud->at(Index).z);
	setClusterAABB(Box);
}

//*****************************************************************
//FUNCTION: 
double CPointCluster4NormalRatio::computeDistanceV(pcl::index_t vPointIndex) const
{
	const auto pCloud = CPointCloudAutoRetouchScene::getInstance()->getPointCloudScene();
	_ASSERTE(pCloud != nullptr);
	_ASSERTE(vPointIndex < pCloud->size());

	const Eigen::RowVector3f Normal = pCloud->at(vPointIndex).getNormalVector3fMap();

	const auto NormalMap = pCloud->getMatrixXfMap(3, sizeof(PointCloud_t::PointType) / sizeof(float), 
		offsetof(PointCloud_t::PointType, data_n) / sizeof(float));
	//TODO: 配置文件添加法线判定为相似的阈值
	//dot(theta) <= cos (20 degrees)
	const auto NormalCount = ((Normal * NormalMap).array() >= 0.93969262078591f).count();

	return static_cast<double>(NormalCount) / static_cast<double>(pCloud->size());
}
