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

			CNeighborhood* buildNeighborhood(pcl::index_t vSeed, const std::string& vBuilderSig);
			CNeighborhood* buildNeighborhood(pcl::index_t vSeed, const pcl::Indices& vRestrictedSet, const std::string& vBuilderSig);

			bool undoLastOp();

			void recordCurrentOp(IOpResult* vResult);
			void init(pcl::PointCloud<pcl::PointSurfel>::Ptr vPointCloudScene);

			std::size_t getNumPoint() const { _ASSERTE(m_pPointCloudScene); return m_pPointCloudScene->size(); }	
#ifdef _UNIT_TEST
            std::size_t getNumOfResultQueue() const { return m_OpResultQueue.getNumOfResultQueue(); }
#endif// _UNIT_TEST 
			const auto& getPointCloudScene() const { return m_pPointCloudScene; }
			const auto& getGlobalKdTree() const { return m_pGlobalKdTree; }
			const auto& getSceneAABB() const { return m_PointCloudSceneAABB; }

			CGlobalPointLabelSet* fetchPointLabelSet() { return &m_PointLabelSet; }
			void resetLabelSet();
		
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


