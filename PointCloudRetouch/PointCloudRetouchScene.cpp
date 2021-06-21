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

Eigen::Vector4d CPointCloudRetouchScene::getPositionAt(pcl::index_t vIndex)
{
	_ASSERTE(vIndex < m_pPointCloudScene->size());
	return { m_pPointCloudScene->points[vIndex].x, m_pPointCloudScene->points[vIndex].y, m_pPointCloudScene->points[vIndex].z, 1.0 };
}

Eigen::Vector4d CPointCloudRetouchScene::getNormalAt(pcl::index_t vIndex)
{
	_ASSERTE(vIndex < m_pPointCloudScene->size());
	return { m_pPointCloudScene->points[vIndex].normal_x, m_pPointCloudScene->points[vIndex].normal_y, m_pPointCloudScene->points[vIndex].normal_z, 0.0 };
}

Eigen::Vector4i CPointCloudRetouchScene::getColorAt(pcl::index_t vIndex)
{
	_ASSERTE(vIndex < m_pPointCloudScene->size());
	return __extractRgba(m_pPointCloudScene->points[vIndex].rgb);
}

Eigen::Vector4i CPointCloudRetouchScene::__extractRgba(float vRgba)
{
	union ColorLayout
	{
		struct
		{
			std::uint8_t b;
			std::uint8_t g;
			std::uint8_t r;
			std::uint8_t a;
		};
		float rgba;
	};
	ColorLayout Color;
	Color.rgba = vRgba;

	Eigen::Vector4i Rgba;
	Rgba.x() = Color.r;
	Rgba.y() = Color.g;
	Rgba.z() = Color.b;
	Rgba.w() = 255;

	return Rgba;
}

