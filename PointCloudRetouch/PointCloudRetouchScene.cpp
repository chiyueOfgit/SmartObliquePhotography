#include "pch.h"
#include "PointCloudRetouchScene.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

CPointCloudRetouchScene::CPointCloudRetouchScene() 
{

}

CPointCloudRetouchScene::~CPointCloudRetouchScene()
{
}

//*****************************************************************
//FUNCTION: 
void CPointCloudRetouchScene::init(PointCloud_t::Ptr vPointCloudScene)
{
	_ASSERTE(vPointCloudScene);
	m_pPointCloudScene = vPointCloudScene;
}

Eigen::Vector4d CPointCloudRetouchScene::getPositionAt(std::size_t vIndex)
{
	_ASSERTE(vIndex < m_pPointCloudScene->size());
	return { m_pPointCloudScene->points[vIndex].x, m_pPointCloudScene->points[vIndex].y, m_pPointCloudScene->points[vIndex].z, 1.0 };
}

Eigen::Vector4d CPointCloudRetouchScene::getNormalAt(std::size_t vIndex)
{
	_ASSERTE(vIndex < m_pPointCloudScene->size());
	return { m_pPointCloudScene->points[vIndex].normal_x, m_pPointCloudScene->points[vIndex].normal_y, m_pPointCloudScene->points[vIndex].normal_z, 0.0 };
}

Eigen::Vector4i CPointCloudRetouchScene::getColorAt(std::size_t vIndex)
{
	_ASSERTE(vIndex < m_pPointCloudScene->size());
	return __extractRgba(m_pPointCloudScene->points[vIndex].rgb);
}


