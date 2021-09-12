#include "pch.h"
#include "PointCloudRetouchScene.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

CPointCloudScene::CPointCloudScene() 
{

}

CPointCloudScene::~CPointCloudScene()
{
}

//*****************************************************************
//FUNCTION: 
void CPointCloudScene::init(const std::vector<PointCloud_t::Ptr>& vTileSet)
{
	m_NumPoints = 0;
	m_TileSet.clear();
	for (int Offset = 0; auto pCloud : vTileSet)
	{
		m_TileSet.push_back({ Offset, pCloud });
		Offset += pCloud->size();
		m_NumPoints += pCloud->size();
	}
}

//*****************************************************************
//FUNCTION: 
Eigen::Vector4f CPointCloudScene::getPositionAt(pcl::index_t vIndex) const
{
	auto Point = __getPoint(vIndex);
	return { Point.x, Point.y, Point.z, 1.0 };
}

//*****************************************************************
//FUNCTION: 
Eigen::Vector4f CPointCloudScene::getNormalAt(pcl::index_t vIndex) const
{
	auto Point = __getPoint(vIndex);
	return { Point.normal_x, Point.normal_y, Point.normal_z, 0.0 };
}

//*****************************************************************
//FUNCTION: 
Eigen::Vector3i CPointCloudScene::getColorAt(pcl::index_t vIndex) const
{
	return __extractRgba(__getPoint(vIndex).rgb);
}

//*****************************************************************
//FUNCTION: 
std::size_t CPointCloudScene::getTileIndexByPoint(pcl::index_t vIndex) const
{
	std::size_t WhichTile = 0;
	while (WhichTile + 1 < m_TileSet.size() && m_TileSet[WhichTile + 1].first <= vIndex)
		WhichTile++;
	return WhichTile;
}

inline PointCloud_t::PointType CPointCloudScene::__getPoint(pcl::index_t vIndex) const
{
	std::size_t WhichTile = 0;
	while (WhichTile + 1 < m_TileSet.size() && m_TileSet[WhichTile + 1].first <= vIndex)
		WhichTile++;

	return m_TileSet[WhichTile].second->points[vIndex - m_TileSet[WhichTile].first];
}

//*****************************************************************
//FUNCTION: 
std::pair<Eigen::Vector3f, Eigen::Vector3f> CPointCloudScene::getBoundingBox(const std::vector<pcl::index_t>& vIndices) const
{
	Eigen::Vector3f Min{ FLT_MAX, FLT_MAX, FLT_MAX };
	Eigen::Vector3f Max{ -FLT_MAX, -FLT_MAX, -FLT_MAX };

	auto update = [&](const Eigen::Vector4f& vPos)  //FIXME-010：为什么要传入四维向量？
	{
		for (int i = 0; i < 3; i++)
		{
			if (vPos.data()[i] < Min.data()[i]) Min.data()[i] = vPos.data()[i];
			if (vPos.data()[i] > Max.data()[i]) Max.data()[i] = vPos.data()[i];
		}
	};

	if (vIndices.empty())
	{
		for (int i = 0; i < m_NumPoints; i++)
			update(getPositionAt(i));
	}
	else
	{
		for (auto& Index : vIndices) update(getPositionAt(Index));
	}

	return { Min, Max };
}

//*****************************************************************
//FUNCTION: 
std::vector<pcl::index_t> CPointCloudScene::getPointsInBox(const std::pair<Eigen::Vector3f, Eigen::Vector3f>& vBox, const Eigen::Matrix3f& vRotationMatrix) const
{
	std::vector<pcl::index_t> PointsInBox;
	for (auto Index = 0; Index < m_NumPoints; Index++)
	{
		auto Point = __getPoint(Index);
		Eigen::Vector3f Pos{ Point.x, Point.y, Point.z };  //FIXME-010：确定这个临时变量会不会在每次循环后都被析构（debug和release方式下都要测试）。如果要被频繁析构，这个变量就要放到循环体外
		Pos = vRotationMatrix * Pos;
		
		int i = 0;
		for (i = 0; i < 3; i++)
		{
			if (Pos.data()[i] < vBox.first.data()[i] || Pos.data()[i] > vBox.second.data()[i]) break;
		}
		if (i == 3) PointsInBox.push_back(Index);
	}
	return PointsInBox;
}

//*****************************************************************
//FUNCTION: 
Eigen::Vector3i CPointCloudScene::__extractRgba(float vRgba) const
{//FIXME-010: 函数名说要提取rgb，但返回的是rgb，故意的吗？
	union ColorLayout
	{
		struct
		{
			std::uint8_t b;
			std::uint8_t g;
			std::uint8_t r;
			std::uint8_t a;
		};
		float rgb;
	};
	ColorLayout Color;  
	Color.rgb = vRgba;    //FIXME-010：这种颜色的转化方法很奇怪，给个说明文档

	Eigen::Vector3i Rgb;
	Rgb.x() = Color.r;
	Rgb.y() = Color.g;
	Rgb.z() = Color.b;

	return Rgb;
}
