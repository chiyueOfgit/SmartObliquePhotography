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

			std::size_t getNumPoint() const { _ASSERTE(m_pPointCloudScene); return m_pPointCloudScene->size(); }

			const auto& getPointCloudScene() const { return m_pPointCloudScene; }
			const auto& getGlobalKdTree() const { return m_pGlobalKdTree; }
			const auto& getSceneAABB() const { return m_PointCloudSceneAABB; }

			CGlobalPointLabelSet* fetchPointLabelSet() { return &m_PointLabelSet; }

		private:
			CPointCloudAutoRetouchScene();

			COpResultQueue m_OpResultQueue;
			CGlobalPointLabelSet m_PointLabelSet;

			pcl::PointCloud<pcl::PointSurfel>::Ptr m_pPointCloudScene = nullptr;
			pcl::search::KdTree<pcl::PointSurfel>::Ptr m_pGlobalKdTree = nullptr;

			SBox m_PointCloudSceneAABB;

		friend class hiveDesignPattern::CSingleton<CPointCloudAutoRetouchScene>;
		};
	}
}


