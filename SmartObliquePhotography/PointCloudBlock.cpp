#include "pch.h"
#include "PointCloudBlock.h"
#include "PCLPointCloudWrapper.h"

using namespace hiveObliquePhotography;

CPointCloudBlock::~CPointCloudBlock()
{

}

//*****************************************************************
//FUNCTION: 
bool CPointCloudBlock::__isValidBlock() const
{
	if (m_PointPositionSet.empty() || m_FirstPointIndexInEachCell.empty() || (m_PointPositionSet.size() <= m_FirstPointIndexInEachCell[m_FirstPointIndexInEachCell.size()-1])) return false;
	if (m_PointPositionSet.size() != m_PointColorSet.size()) return false;
	if (!m_PointNormalSet.empty() && (m_PointPositionSet.size() != m_PointNormalSet.size())) return false;
	if (!m_PointCategorySet.empty() && (m_PointPositionSet.size() != m_PointCategorySet.size())) return false;
	if (m_FirstPointIndexInEachCell.size() != m_CellDimX * m_CellDimY) return false;

	for (auto i = 0; i < m_CellDimX * m_CellDimY - 1; i++)
	{
		if (!__isValidCell(m_FirstPointIndexInEachCell[i], m_FirstPointIndexInEachCell[i + 1], i)) return false;
	}
	if (!__isValidCell(m_FirstPointIndexInEachCell[m_FirstPointIndexInEachCell.size()-1], m_PointPositionSet.size(), m_FirstPointIndexInEachCell.size() - 1)) return false;
	return true;
}

//*****************************************************************
//FUNCTION: 
bool CPointCloudBlock::__isValidCell(std::uint32_t vFirstPointIndex, std::uint32_t vLastPointIndex, std::uint32_t vCellOffset) const
{
	_ASSERTE((vCellOffset < m_CellDimX* m_CellDimY) && (vFirstPointIndex <= m_PointPositionSet.size()) && (vLastPointIndex <= m_PointPositionSet.size()));

	std::pair<std::uint32_t, std::uint32_t> CellIndex;
	float PreviousZ = FLT_MAX;
	for (auto Idx = vFirstPointIndex; Idx < vLastPointIndex; Idx++)
	{
		const Eigen::Vector3f& Pos = m_PointPositionSet[Idx];
		if (Pos.z() > PreviousZ) return false;  //确保一个cell内的点，是按照其高程从高到低排序
		PreviousZ = Pos.z();
		CellIndex = __computeCellIndex(Pos);
		if (vCellOffset != _OFFSET_2D(CellIndex.first, CellIndex.second, m_CellDimX)) return false;  //确保每个点放在其所属的cell内 
	}
	return true;
}

//*****************************************************************
//FUNCTION: 
void CPointCloudBlock::addPoint(const Eigen::Vector3f& vPosition, const SPointColor& vColor)
{
	m_PointPositionSet.emplace_back(vPosition);
	m_PointColorSet.emplace_back(vColor);
}

//*****************************************************************
//FUNCTION: 
void CPointCloudBlock::save()
{

}

//*****************************************************************
//FUNCTION: 
void CPointCloudBlock::unload()
{//这里要unload哪些内容需要慎重，比如m_FirstPointIndexInEachCell，如果这里unload了，以后重新载入的时候需要重新生成，到底是用更多的内存开销还是消耗更多的CPU时间，需要考虑 
	m_PointPositionSet.clear();
	m_PointNormalSet.clear();
	m_PointColorSet.clear();
}

//*****************************************************************
//FUNCTION: 
bool CPointCloudBlock::buildPCLPointCloud(std::uint8_t vBuildFlag, bool vDestroyOriginalData)
{
	if (!m_pPCLPointCloud) m_pPCLPointCloud = new CPCLPointCloudWrapper;
	if (!m_pPCLPointCloud->init(vBuildFlag)) return false;
	if (vDestroyOriginalData)
	{
//根据传入的vBuildFlag，来清空相应的数据，以减少内存开销
	}
	return true;
}