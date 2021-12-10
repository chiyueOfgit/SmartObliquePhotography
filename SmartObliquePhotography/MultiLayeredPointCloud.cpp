#include "pch.h"
#include "MultiLayeredPointCloud.h"

using namespace hiveObliquePhotography;

CMultiLayeredPointCloud::~CMultiLayeredPointCloud()
{

}

//*****************************************************************
//FUNCTION: 
void CMultiLayeredPointCloud::init(float vLongitudeSpan, float vLatitudeSpan, float vCellScale, unsigned char vExpectedBlockSizeMB)
{

}

//*****************************************************************
//FUNCTION: 
void CMultiLayeredPointCloud::addPointSet(const SUnorganizedPointCloud* vInputPointCloud)
{
	_ASSERTE(vInputPointCloud && !vInputPointCloud->isValid() && !m_BlockSet.empty() && (m_BlockDimX > 0) && (m_BlockDimY > 0));

	std::pair<std::uint32_t, std::uint32_t> BlockIdx;
	for (auto i = 0; i < vInputPointCloud->_PositionSet.size(); i++)
	{
		BlockIdx = __computeBlockIndex(vInputPointCloud->_PositionSet[i]);
		_ASSERTE(__isValidBlockIndex(BlockIdx));
		std::uint32_t BlockOffset = _OFFSET_2D(BlockIdx.first, BlockIdx.second, m_BlockDimX);
		_ASSERTE(BlockOffset < m_BlockSet.size());
		m_BlockSet[BlockOffset].addPoint(vInputPointCloud->_PositionSet[i], vInputPointCloud->_ColorSet[i]);
	}
}