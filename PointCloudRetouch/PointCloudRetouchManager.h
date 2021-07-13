#pragma once
#include "PointCloudRetouchScene.h"
#include "PointClusterSet.h"
#include "PointLabelSet.h"
#include "RetouchTask.h"
#include "InitialClusterCreator.h"
#include "PointSetPreprocessor.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CPointCluster;
		class INeighborhoodBuilder;

		class CPointCloudRetouchManager : public hiveDesignPattern::CSingleton<CPointCloudRetouchManager>
		{
		public:
			~CPointCloudRetouchManager() = default;

			std::vector<pcl::index_t> buildNeighborhood(pcl::index_t vSeed, std::uint32_t vSeedClusterIndex);
			void tagPointLabel(pcl::index_t vPoint, EPointLabel vTargetLabel, std::uint32_t vClusterIndex, double vClusterBelongingProbability)
			{
				m_PointLabelSet.tagPointLabel(vPoint, vTargetLabel, vClusterIndex, vClusterBelongingProbability);
			}
			bool dumpPointLabel(std::vector<std::size_t>& voPointLabel) const;

			void dumpExpandPoints(std::vector<int>& voExpandPoints, bool vIsLitterMarker) 
			{
				vIsLitterMarker ? m_LitterMarker.dumpTaskMarkedPoints(voExpandPoints) : m_BackgroundMarker.dumpTaskMarkedPoints(voExpandPoints);
			}

			bool init(PointCloud_t::Ptr vPointCloud, const hiveConfig::CHiveConfig* vConfig);
			void clearMark();
			bool executeMarker(const std::vector<pcl::index_t>& vUserMarkedRegion, double vHardness, double vRadius, const Eigen::Vector2f& vCenter, const Eigen::Matrix4d& vPvMatrix, const std::pair<float, float>& vWindowSize, EPointLabel vTargetLabel);
			bool executeOutlierDetector(EPointLabel vTo);
			void recordCurrentStatus();
			bool undo();
			
			std::size_t   getNumCluster() const { return m_PointClusterSet.getNumCluster(); }
			std::uint32_t addAndGetTimestamp() { m_Timestamp++; return m_Timestamp; }
			std::uint32_t getClusterIndexAt(std::size_t vIndex) const { return m_PointLabelSet.getClusterIndexAt(vIndex); }

			double getClusterBelongingProbabilityAt(std::size_t vIndex) const { return m_PointLabelSet.getClusterBelongingProbabilityAt(vIndex); }

			void switchLabel(EPointLabel vTo, EPointLabel vFrom);
			void setLabel(const std::vector<pcl::index_t>& vPoints, EPointLabel vTarget);	//for perform
			void dumpIndicesByLabel(std::vector<pcl::index_t>& vioIndices, EPointLabel vLabel);
			
			const auto& getRetouchScene() const { return m_Scene; }
			
#ifdef _UNIT_TEST
			const auto& getOutlierConfig() const { return m_pOutlierConfig; }
			const auto& getClusterSet() const { return m_PointClusterSet; }
			const auto& getLabelSet() const { return m_PointLabelSet; }
			const auto& getLitterMarker() const { return m_LitterMarker; }
			const auto& getBackgroundMarker() const { return m_BackgroundMarker; }
			//CPointCluster* generateInitialCluster(const std::vector<pcl::index_t>& vUserMarkedRegion, double vHardness, double vRadius, const Eigen::Vector2f& vCenter, const Eigen::Matrix4d& vPvMatrix, const std::pair<float, float>& vWindowSize, EPointLabel vTargetLabel)
			//{
			//	return __generateInitialCluster(vUserMarkedRegion, vHardness, vRadius, vCenter, vPvMatrix, vWindowSize, vTargetLabel);
			//}
#endif // _UNIT_TEST


		private:
			CPointCloudRetouchManager() {};

			std::uint32_t m_Timestamp = 0;

			CPointClusterSet         m_PointClusterSet;
			CPointLabelSet           m_PointLabelSet;
			CPointCloudRetouchScene  m_Scene;
			CRetouchTask             m_LitterMarker;
			CRetouchTask             m_BackgroundMarker;
			CInitialClusterCreator   m_InitialClusterCreator;
			CPointSetPreprocessor    m_Preprocessor;
			INeighborhoodBuilder    *m_pNeighborhoodBuilder = nullptr;
			const hiveConfig::CHiveConfig* m_pConfig = nullptr;
			const hiveConfig::CHiveConfig* m_pOutlierConfig = nullptr;
			
			std::deque<std::pair<CPointLabelSet, std::uint32_t>> m_StatusQueue;
			CPointCluster* __generateInitialCluster(const std::vector<pcl::index_t>& vUserMarkedRegion, double vHardness, double vRadius, const Eigen::Vector2f& vCenter, const Eigen::Matrix4d& vPvMatrix, const std::pair<float, float>& vWindowSize, EPointLabel vTargetLabel);

		friend class hiveDesignPattern::CSingleton<CPointCloudRetouchManager>;
		};
	}
}