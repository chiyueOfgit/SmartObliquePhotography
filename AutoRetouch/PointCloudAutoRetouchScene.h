#pragma once
#include "AutoRetouchCommon.h"
#include "OpResultQueue.h"
#include "PointLabel4Classfier.h"

namespace hiveObliquePhotography
{
	namespace AutoRetouch
	{
		class CNeighborhood;
		class IPointCluster;

		class CPointCloudAutoRetouchScene : public hiveDesignPattern::CSingleton<CPointCloudAutoRetouchScene>
		{
		public:
			~CPointCloudAutoRetouchScene();

			CNeighborhood* buildNeighborhood(std::uint64_t vSeed, const std::string& vBuilderSig);
			CNeighborhood* buildNeighborhood(std::uint64_t vSeed, const std::vector<std::uint64_t>& vRestrictedSet, const std::string& vBuilderSig);

			bool undoLastOp();

			void recordCurrentOp(IOpResult* vResult);
			void init(pcl::PointCloud<pcl::PointSurfel>::Ptr vPointCloudScene);

			void addPointCluster(IPointCluster* vPointCluster) { m_pPointClusterSet.push_back(vPointCluster); }

			const std::vector<IPointCluster*>& getPointClusters() const { return m_pPointClusterSet; }

			std::size_t getNumPoint() const { _ASSERTE(m_pPointCloudScene); return m_pPointCloudScene->size(); }

			const auto& getPointCloudScene() const { return m_pPointCloudScene; }
			const auto& getGlobalKdTree() const { return m_pGlobalKdTree; }

			CGlobalPointLabelSet* fetchPointLabelSet() { return &m_PointLabelSet; }

		private:
			CPointCloudAutoRetouchScene();

			COpResultQueue m_OpResultQueue;
			CGlobalPointLabelSet m_PointLabelSet;
			std::vector<IPointCluster*> m_pPointClusterSet;

			pcl::PointCloud<pcl::PointSurfel>::Ptr m_pPointCloudScene = nullptr;
			pcl::search::KdTree<pcl::PointSurfel>::Ptr m_pGlobalKdTree = nullptr;

		friend class hiveDesignPattern::CSingleton<CPointCloudAutoRetouchScene>;
		};
	}
}


