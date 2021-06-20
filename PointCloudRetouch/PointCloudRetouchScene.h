#pragma once
#include "PointClusterSet.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CPointCloudRetouchScene 
		{
		public:
			CPointCloudRetouchScene();
			~CPointCloudRetouchScene();

			void buildNeighborhood(pcl::index_t vSeed, const std::string& vBuilderSig, std::vector<pcl::index_t>& voNeighborhood);
			void buildNeighborhood(pcl::index_t vSeed, const pcl::Indices& vRestrictedSet, const std::string& vBuilderSig, std::vector<pcl::index_t>& voNeighborhood);

			void init(PointCloud_t::Ptr vPointCloudScene);
			void reset(std::uint64_t vTimestamp);

			std::size_t getNumPoint() const { _ASSERTE(m_pPointCloudScene); return m_pPointCloudScene->size(); }	

		private:
			PointCloud_t::Ptr m_pPointCloudScene = nullptr;
			pcl::search::KdTree<PointCloud_t::PointType>::Ptr m_pGlobalKdTree = nullptr;

		friend class hiveDesignPattern::CSingleton<CPointCloudRetouchScene>;
		};
	}
}


