#pragma once
#include <vector>
#include <string>
#include <Eigen/Core>
#include "SmartOPCommon.h"

namespace hiveObliquePhotography
{
	class CPCLPointCloudWrapper;

	class CPointCloudBlock
	{
	public:
		CPointCloudBlock();
		~CPointCloudBlock();

		void tagPointsInCellAs(std::uint32_t vCellX, std::uint32_t vCellY, EPointCategory vCategory);
		void init(std::uint32_t vBlockX, std::uint32_t vBlockY, float vCellScale, const std::vector<std::uint32_t>& vTileContributerSet);
		void addPoint(const Eigen::Vector3f& vPosition, const SPointColor& vColor);
		void save();  //make sure this function is executed in async way
		void resetPointCategory();
		void unload();
		void destroyPCLPointCloud();

		[[nodiscard]] bool buildPCLPointCloud(std::uint8_t vBuildFlag, bool vDestroyOriginalData);
		[[nodiscard]] bool load(std::uint8_t vLoadFlag);   //make sure this function is executed in async way

	private:
		bool m_IsPointSorted = false;
		std::uint32_t m_BlockX = INVALID_BLOCK_COORD;
		std::uint32_t m_BlockY = INVALID_BLOCK_COORD;
		std::uint32_t m_CellDimX = 0;
		std::uint32_t m_CellDimY = 0;
		float m_CellScale;
		unsigned short m_BlockCategory;  //用于标记当前block的点云有哪些类型 （参见EPointCategory）
		CPCLPointCloudWrapper* m_pPCLPointCloud = nullptr;
		std::vector<Eigen::Vector3f> m_PointPositionSet;
		std::vector<Eigen::Vector3f> m_PointNormalSet;
		std::vector<SPointColor>     m_PointColorSet;
		std::vector<EPointCategory>  m_PointCategorySet;
		std::vector<std::uint32_t>   m_FirstPointIndexInEachCell;  //same size as number of cells
		
		std::string __generatePointBlockFileName();

		std::pair<std::uint32_t, std::uint32_t> __computeCellIndex(const Eigen::Vector3f& vPos) const;

		[[nodiscard]] bool __isValidBlock() const;
		[[nodiscard]] bool __isValidCell(std::uint32_t vFirstPointIndex, std::uint32_t vLastPointIndex, std::uint32_t vCellOffset) const;
	};
}

