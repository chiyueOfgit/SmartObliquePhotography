#include "pch.h"
#include "ElevationMapGenerator.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION:
bool CElevationMapGenerator::execute(const Eigen::Vector2i vResolution, const std::vector<pcl::index_t>& vPointIndexSet)
{
	Eigen::Matrix<float, -1, -1> Texture(vResolution.x(), vResolution.y());
	generateDistributionSet(vResolution, vPointIndexSet);
	for (int i = 0; i < vResolution.x(); i++)
		for (int k = 0; k < vResolution.y(); k++)
			Texture(k, i) = __transElevation2Color(m_HeightSet[k][i] - m_Box.first.z(), m_Box.second.z() - m_Box.first.z());

	m_ElevationMap.fillColor(vResolution.y(), vResolution.x(), Texture.data());
	return true;
}

//*****************************************************************
//FUNCTION:
bool CElevationMapGenerator::generateDistributionSet(const Eigen::Vector2i vResolution, const std::vector<pcl::index_t>& vPointIndexSet)
{
	m_PointDistributionSet.clear();
	m_Box = CPointCloudRetouchManager::getInstance()->getScene().getBoundingBox(std::vector<pcl::index_t>());

	Eigen::Vector2f Offset{ (m_Box.second - m_Box.first).x() / vResolution.x(), (m_Box.second - m_Box.first).y() / vResolution.y() };
	m_HeightSet.resize(vResolution.y(), std::vector<float>(vResolution.x(), m_Box.first.z()));
	Eigen::Vector2f MinXY{ m_Box.first.x(),m_Box.first.y() };
	__calcAreaElevation(MinXY, Offset, vPointIndexSet);
	return true;
}

//*****************************************************************
//FUNCTION:
void CElevationMapGenerator::__calcAreaElevation(const Eigen::Vector2f& vMinCoord, const Eigen::Vector2f& vOffset, const std::vector<pcl::index_t>& vPointIndexSet)
{
	const float Radius = 0.3;

	m_PointDistributionSet.resize(m_HeightSet.size(), std::vector<std::vector<pcl::index_t>>(m_HeightSet[0].size()));
	auto Scene = CPointCloudRetouchManager::getInstance()->getScene();
	for (auto PointIndex : vPointIndexSet)
	{
		auto Position = Scene.getPositionAt(PointIndex);

		int RowBegin = (Position.y() - vMinCoord.y() - Radius) / vOffset.y();
		int RowEnd = (Position.y() - vMinCoord.y() + Radius) / vOffset.y();
		int ColBegin = (Position.x() - vMinCoord.x() - Radius) / vOffset.x();
		int ColEnd = (Position.x() - vMinCoord.x() + Radius) / vOffset.x();
		int RowRaw = (Position.y() - vMinCoord.y()) / vOffset.y();
		int ColRaw = (Position.x() - vMinCoord.x()) / vOffset.x();

		if (RowBegin > m_HeightSet.size()) RowBegin = m_HeightSet.size();
		if (RowEnd > m_HeightSet.size()) RowEnd = m_HeightSet.size();
		if (ColBegin > m_HeightSet[0].size()) ColBegin = m_HeightSet[0].size();
		if (ColEnd > m_HeightSet[0].size()) ColEnd = m_HeightSet[0].size();
		if (RowRaw == m_HeightSet.size()) RowRaw = RowRaw - 1;
		if (ColRaw == m_HeightSet[0].size()) ColRaw = ColRaw - 1;
		m_PointDistributionSet[RowRaw][ColRaw].push_back(PointIndex);

		for (int i = ColBegin; i < ColEnd; i++)
			for (int k = RowBegin; k < RowEnd; k++)
				if (Position.z() > m_HeightSet[k][i])
					m_HeightSet[k][i] = Position.z();
	}
}

//*****************************************************************
//FUNCTION:
float CElevationMapGenerator::__transElevation2Color(float vElevation, float vHeightDiff)
{
	float Percentage = vElevation / vHeightDiff;
	return { Percentage * 255 };
}

//*****************************************************************
//FUNCTION:
bool CElevationMapGenerator::dumpElevationMap(CImage<float>& voElevationMap)
{
	if (m_ElevationMap.getHeight() && m_ElevationMap.getWidth())
	{
		voElevationMap = m_ElevationMap;
		return true;
	}
	else
		return false;
		
}

//*****************************************************************
//FUNCTION:
bool CElevationMapGenerator::dumpPointDistributionSet(std::vector<std::vector<std::vector<pcl::index_t>>>& voPointDistributionSet)
{
	if (m_PointDistributionSet.size())
	{
		voPointDistributionSet = m_PointDistributionSet;
		return true;
	}
	else
		return false;
		
}