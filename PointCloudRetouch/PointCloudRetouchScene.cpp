#include "pch.h"
#include "PointCloudRetouchScene.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

CPointCloudRetouchScene::CPointCloudRetouchScene() : m_pGlobalKdTree(new pcl::search::KdTree<PointCloud_t::PointType>)
{

}

CPointCloudRetouchScene::~CPointCloudRetouchScene()
{
	m_pGlobalKdTree.reset();
}

//*****************************************************************
//FUNCTION: 
void CPointCloudRetouchScene::init(PointCloud_t::Ptr vPointCloudScene)
{
	_ASSERTE(vPointCloudScene && m_pGlobalKdTree);
	m_pPointCloudScene = vPointCloudScene;
	m_pGlobalKdTree->setInputCloud(m_pPointCloudScene);
}