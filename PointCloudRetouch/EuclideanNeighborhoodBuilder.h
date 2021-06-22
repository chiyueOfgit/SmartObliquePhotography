#pragma once
#include "NeighborhoodBuilder.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CEuclideanNeighborhoodBuilder : public INeighborhoodBuilder
		{
		public:
			CEuclideanNeighborhoodBuilder() = default;
			~CEuclideanNeighborhoodBuilder() = default;

		private:
			virtual void __extraInitV(const hiveConfig::CHiveConfig* vConfig) override;
			virtual void __buildNeighborhoodV(pcl::index_t vSeed, std::vector<pcl::index_t>& voNeighborhood) override;

			pcl::search::KdTree<pcl::PointSurfel>::Ptr m_pTree = nullptr;

			std::string m_SearchMode;
			int m_NearestN;
			float m_Radius;
		};
	}
}
