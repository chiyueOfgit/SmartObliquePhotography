#pragma once
#include "Image.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CElevationMapGenerator
		{
		public:
			CElevationMapGenerator() = default;
			~CElevationMapGenerator() = default;

			bool execute(const Eigen::Vector2i vResolution, const std::vector<pcl::index_t>& vPointIndexSet);
			bool generateDistributionSet(const Eigen::Vector2i vResolution, const std::vector<pcl::index_t>& vPointIndexSet);

			bool dumpElevationMap(CImage<float>& voElevationMap);
			bool dumpPointDistributionSet(std::vector<std::vector<std::vector<pcl::index_t>>>& voPointDistributionSet);

		private:
			CImage<float> m_ElevationMap;
			std::pair<Eigen::Vector3f, Eigen::Vector3f> m_Box;
			std::vector<std::vector<std::vector<pcl::index_t>>> m_PointDistributionSet;
			std::vector<std::vector<float>> m_HeightSet;

			void __calcAreaElevation(const Eigen::Vector2f& vMinCoord, const Eigen::Vector2f& vOffset, const std::vector<pcl::index_t>& vPointIndexSet);
			float __transElevation2Color(float vElevation, float vHeightDiff);
		};
	}
}