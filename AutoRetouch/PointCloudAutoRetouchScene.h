#pragma once
#include "AutoRetouchCommon.h"

namespace hiveObliquePhotography
{
	class CNeighborhood;

	namespace AutoRetouch
	{
		class CPointCloudAutoRetouchScene : public hiveDesignPattern::CSingleton<CPointCloudAutoRetouchScene>
		{
		public:
			~CPointCloudAutoRetouchScene();

			CNeighborhood* buildNeighborhood(std::uint64_t vSeed, const std::string& vBuilderSig);
			CNeighborhood* buildNeighborhood(std::uint64_t vSeed, const std::vector<std::uint64_t>& vRestrictedSet, const std::string& vBuilderSig);

		private:
			CPointCloudAutoRetouchScene();

			pcl::PointCloud<pcl::PointSurfel>* m_pPointCloudScenen = nullptr;
			pcl::search::KdTree<pcl::PointSurfel>* m_pGlobalKdTree = nullptr;

		friend class hiveDesignPattern::CSingleton<CPointCloudAutoRetouchScene>;
		};
	}
}


