#pragma once
#include "RegionGrowingAlg.h"
#include "Common.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		using PointCloud_t = pcl::PointCloud<pcl::PointSurfel>;
		
		//TODO: Ìí¼ÓUNKNOWN£¿
		enum class EColorMode : unsigned char
		{
			MEAN = 0,
			MEDIAN,
		};

		class CRegionGrowingByColorAlg :public CRegionGrowingAlg
		{
		public:
			CRegionGrowingByColorAlg() = default;
			~CRegionGrowingByColorAlg() override = default;

		private:
			EColorMode m_ColorTestMode{};
			bool m_EnableColorTest{};
			bool m_EnableGroundTest{};
			bool m_EnableNormalTest{};

			std::uint32_t m_SeedsAverageColor;
			std::size_t m_SeedsSize;
			mutable unsigned int m_AverageColor[4];
			mutable std::uint32_t m_MedianColor;
			mutable std::vector<std::uint32_t> m_MortonCodes;
			
			void __initValidation(const pcl::Indices& vSeeds, PointCloud_t::ConstPtr vCloud) override;
			bool __validatePointV(pcl::index_t vTestIndex, PointCloud_t::ConstPtr vCloud) const override;
			
			float __calculateColorimetricalDifference(std::uint32_t vFirstColor[3], std::uint32_t vSecondColor[3]) const;

			bool __colorTestByAverage(int vTestIndex, PointCloud_t::ConstPtr vCloud) const;
			bool __colorTestByMedian(int vTestIndex, PointCloud_t::ConstPtr vCloud) const;
			bool __groundTest(int vTestIndex, PointCloud_t::ConstPtr vCloud) const;
			bool __normalTest(int vTestIndex, PointCloud_t::ConstPtr vCloud) const;

			//TODO: move to common?
			static std::uint32_t __morton4(std::uint32_t vData);
			static std::uint32_t __inverseMorton4(std::uint32_t vMorton);
			static std::tuple<std::uint8_t, std::uint8_t, std::uint8_t, std::uint8_t> __extractColor(std::uint32_t vColor);
		};
	}
}