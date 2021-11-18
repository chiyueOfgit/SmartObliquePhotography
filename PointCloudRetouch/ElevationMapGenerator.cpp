#include "pch.h"
#include "ElevationMapGenerator.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION:
bool CElevationMapGenerator::execute(const Eigen::Vector2i vResolution, const std::vector<pcl::index_t>& vPointIndexSet)
{
	Eigen::Matrix<float, -1, -1> Texture(vResolution.x(), vResolution.y());
	generateDistributionSet(vResolution, vPointIndexSet);
	std::vector<std::vector<float>> HeightSet(vResolution.y(), std::vector<float>(vResolution.x(), m_Box.first.z()));
	for (int i = 0; i < vResolution.x(); i++)
		for (int k = 0; k < vResolution.y(); k++)
			Texture(k, i) = __transElevation2Color(HeightSet[k][i] - m_Box.first.z(), m_Box.second.z() - m_Box.first.z());

	m_ElevationMap.fillColor(vResolution.y(), vResolution.x(), Texture.data());
}

//*****************************************************************
//FUNCTION:
bool CElevationMapGenerator::generateDistributionSet(const Eigen::Vector2i vResolution, const std::vector<pcl::index_t>& vPointIndexSet)
{
	m_Box = CPointCloudRetouchManager::getInstance()->getScene().getBoundingBox(std::vector<pcl::index_t>());

	Eigen::Vector2f Offset{ (m_Box.second - m_Box.first).x() / vResolution.x(), (m_Box.second - m_Box.first).y() / vResolution.y() };
	std::vector<std::vector<float>> HeightSet(vResolution.y(), std::vector<float>(vResolution.x(), m_Box.first.z()));
	Eigen::Vector2f MinXY{ m_Box.first.x(),m_Box.first.y() };
	__calcAreaElevation(MinXY, Offset, HeightSet, vPointIndexSet);
}

//*****************************************************************
//FUNCTION:
void CElevationMapGenerator::__calcAreaElevation(const Eigen::Vector2f& vMinCoord, const Eigen::Vector2f& vOffset, std::vector<std::vector<float>>& vioHeightSet, const std::vector<pcl::index_t>& vPointIndexSet)
{
	const float Radius = 0.3;
	m_PointDistributionSet.resize(vioHeightSet.size(), std::vector<std::vector<pcl::index_t>>(vioHeightSet[0].size()));
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

		if (RowBegin > vioHeightSet.size()) RowBegin = vioHeightSet.size();
		if (RowEnd > vioHeightSet.size()) RowEnd = vioHeightSet.size();
		if (ColBegin > vioHeightSet[0].size()) ColBegin = vioHeightSet[0].size();
		if (ColEnd > vioHeightSet[0].size()) ColEnd = vioHeightSet[0].size();
		if (RowRaw == vioHeightSet.size()) RowRaw = RowRaw - 1;
		if (ColRaw == vioHeightSet[0].size()) ColRaw = ColRaw - 1;
		m_PointDistributionSet[RowRaw][ColRaw].push_back(PointIndex);

		for (int i = ColBegin; i < ColEnd; i++)
			for (int k = RowBegin; k < RowEnd; k++)
				if (Position.z() > vioHeightSet[k][i])
					vioHeightSet[k][i] = Position.z();
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
	voElevationMap = m_ElevationMap;
	return true;
}

//*****************************************************************
//FUNCTION:
bool CElevationMapGenerator::dumpPointDistributionSet(std::vector<std::vector<std::vector<pcl::index_t>>>& voPointDistributionSet)
{
	voPointDistributionSet = m_PointDistributionSet;
	return true;
}