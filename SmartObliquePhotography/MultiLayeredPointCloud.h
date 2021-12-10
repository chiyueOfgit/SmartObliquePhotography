#pragma once
#include "SmartOPCommon.h"
#include "PointCloudBlock.h"

namespace hiveObliquePhotography
{
	//1. this is only a wrapper class to store point cloud data, the actual class responsible for storage is CPointCloudBlock
	//2. it provides interface to access information from point cloud
	//3. all operations on the point cloud are implemented by other classes
	class CMultiLayeredPointCloud
	{
	public:
		CMultiLayeredPointCloud() = default;
		~CMultiLayeredPointCloud();

		void init(float vLongitudeSpan, float vLatitudeSpan, float vCellScale, unsigned char vExpectedBlockSizeMB);
		void addPointSet(const SUnorganizedPointCloud *vInputPointCloud);

		float getElevationAt(std::uint32_t vCellX, std::uint32_t vCellY);

		std::uint32_t getNumPointsInCell(std::uint32_t vCellX, std::uint32_t vCellY);

	private:
		float  m_LongitudeSpan = 0;
		float  m_LatitudeSpan = 0;
		float  m_OriginX = 0;
		float  m_OriginY = 0;
		float  m_CellScale = 0;
		std::uint32_t m_BlockDimX = 0;
		std::uint32_t m_BlockDimY = 0;
		std::vector<CPointCloudBlock> m_BlockSet;

		[[nodiscard]] bool __saveBlock(std::uint32_t vBlockX, std::uint32_t vBlockY);
		[[nodiscard]] bool __isValidBlockIndex(const std::pair<std::uint32_t, std::uint32_t>& vBlockIdx);

		std::pair<std::uint32_t, std::uint32_t> __computeBlockIndex(const Eigen::Vector3f& vPos);

		friend class CUnorganziedPointCloud2MultiLayeredConverter;
	};
}