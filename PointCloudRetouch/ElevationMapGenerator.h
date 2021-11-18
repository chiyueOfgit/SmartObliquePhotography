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

			void dumpElevationMap(CImage<float>& voElevationMap);
			void dumpPointDistributionSet(std::vector<std::vector<std::vector<pcl::index_t>>>& voPointDistributionSet);

		private:
			CImage<float> m_ElevationMap;
			std::vector<std::vector<std::vector<pcl::index_t>>> m_PointDistributionSet;
		};
	}
}