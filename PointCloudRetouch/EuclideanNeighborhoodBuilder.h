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
			virtual void __extraInitV() override;
			virtual void __buildNeighborhoodV(pcl::index_t vSeed, std::vector<pcl::index_t>& voNeighborhood) override;

			PointCloud_t::Ptr m_pCloud = nullptr;
			pcl::search::KdTree<pcl::PointSurfel>::Ptr m_pTree = nullptr;
		};
	}
}
