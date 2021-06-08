#pragma once
#include "RegionGrowingAlg.h"
#include "Common.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		struct SRegionGrowingSetting
		{
			enum class EColorMode
			{
				Mean,
				Median,
			};
			float GroundHeight = 446.2f;
			bool bColorFlag = true;
			bool bGroundFlag = true;
			bool bNormalFlag = false;
			EColorMode ColorMode = EColorMode::Mean;
			float SearchSize = 0.5f;
			float ColorThreshold = 15.0f;
		};

		class CRegionGrowingByColorAlg :public CRegionGrowingAlg
		{
		public:
			CRegionGrowingByColorAlg() = default;
			~CRegionGrowingByColorAlg() = default;

			void runV(const std::vector<std::uint64_t>& vSeedSet, EPointLabel vSeedLabel) override;

		private:
			mutable std::pair<std::vector<unsigned int>, int> m_AverageColor;
			mutable std::vector<unsigned int> m_MedianColor{ 0,0,0 };
			mutable std::vector<unsigned int> m_MortonCode;

			ColorDifferences m_ColorDifferenceCalculator;

			bool __validatePoint(int vTestPoint, const pcl::PointCloud<pcl::PointSurfel>::Ptr& vCloud) const;
			float __calculateColorimetricalDifference(std::vector<unsigned int>& vFirstColor, std::vector<unsigned int>& vSecondColor) const;

			bool __colorTestByAverage(int vTestPoint, const pcl::PointCloud<pcl::PointSurfel>::Ptr& vCloud) const;
			bool __colorTestByMedian(int vTestPoint, const pcl::PointCloud<pcl::PointSurfel>::Ptr& vCloud) const;
			bool __groundTest(int vTestPoint, const pcl::PointCloud<pcl::PointSurfel>::Ptr& vCloud) const;
			bool __normalTest(int vTestPoint, const pcl::PointCloud<pcl::PointSurfel>::Ptr& vCloud) const;
		};
	}
}