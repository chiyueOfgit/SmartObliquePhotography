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

		using PointCloud_t = pcl::PointCloud<pcl::PointSurfel>;
		class CPointCloudAutoRetouchScene : public hiveDesignPattern::CSingleton<CPointCloudAutoRetouchScene>
		{
		public:
			~CPointCloudAutoRetouchScene();

			CNeighborhood* buildNeighborhood(pcl::index_t vSeed, const std::string& vBuilderSig);
			CNeighborhood* buildNeighborhood(pcl::index_t vSeed, const pcl::Indices& vRestrictedSet, const std::string& vBuilderSig);

			bool undoLastOp();

			void recordCurrentOp(IOpResult* vResult);
			void init(PointCloud_t::Ptr vPointCloudScene);

			std::size_t getNumPoint() const { _ASSERTE(m_pPointCloudScene); return m_pPointCloudScene->size(); }	
#ifdef _UNIT_TEST
            std::size_t getNumOfResultQueue() const { return m_OpResultQueue.getNumOfResultQueue(); }
#endif// _UNIT_TEST 
			const auto& getPointCloudScene() const { return m_pPointCloudScene; }
			const auto& getGlobalKdTree() const { return m_pGlobalKdTree; }
			const auto& getSceneAABB() const { return m_PointCloudSceneAABB; }

			CGlobalPointLabelSet* fetchPointLabelSet() { return &m_PointLabelSet; }
			void resetLabelSet();
			void clearResultQueue();
		
		private:
			CPointCloudAutoRetouchScene();

			COpResultQueue m_OpResultQueue;
			CGlobalPointLabelSet m_PointLabelSet;

			PointCloud_t::Ptr m_pPointCloudScene = nullptr;
			pcl::search::KdTree<PointCloud_t::PointType>::Ptr m_pGlobalKdTree = nullptr;

			SBox m_PointCloudSceneAABB;

		friend class hiveDesignPattern::CSingleton<CPointCloudAutoRetouchScene>;
		};
	}
}


