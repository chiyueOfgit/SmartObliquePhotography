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
		if (Pos.z() > PreviousZ) return false;  //ȷ��һ��cell�ڵĵ㣬�ǰ�����̴߳Ӹߵ�������
		PreviousZ = Pos.z();
		CellIndex = __computeCellIndex(Pos);
		if (vCellOffset != _OFFSET_2D(CellIndex.first, CellIndex.second, m_CellDimX)) return false;  //ȷ��ÿ���������������cell�� 
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
{//����Ҫunload��Щ������Ҫ���أ�����m_FirstPointIndexInEachCell���������unload�ˣ��Ժ����������ʱ����Ҫ�������ɣ��������ø�����ڴ濪���������ĸ����CPUʱ�䣬��Ҫ���� 
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
//���ݴ����vBuildFlag���������Ӧ�����ݣ��Լ����ڴ濪��
	}
	return true;
}