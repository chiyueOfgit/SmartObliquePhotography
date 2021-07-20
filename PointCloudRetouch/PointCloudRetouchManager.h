#pragma once
#include "PointCloudRetouchScene.h"
#include "PointClusterSet.h"
#include "PointLabelSet.h"
#include "RetouchTask.h"
#include "InitialClusterCreator.h"
#include "PointSetPreprocessor.h"
#include "PrecomputeManager.h"

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
			bool executePreprocessor(std::vector<pcl::index_t>& vioPointSet, const Eigen::Matrix4d& vPvMatrix, const std::function<double(Eigen::Vector2d)>& vSignedDistanceFunc, const Eigen::Vector3d& vViewPos);
			bool executeMarker(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix, const std::function<double(Eigen::Vector2d)>& vHardnessFunc, EPointLabel vTargetLabel);
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

			bool dumpColorFeatureMainColors(std::vector<Eigen::Vector3i>& vMainColors) const;
			bool dumpColorFeatureNearestPoints(std::vector<pcl::index_t>& vNearestPoints) const;
			
			const auto& getRetouchScene() const { return m_Scene; }
			const auto& getPrecomputeManager() const { return m_pPrecomputeManager; }

#ifdef _UNIT_TEST
			void reset4UnitTest() { __reset(); }
			const auto& getOutlierConfig() const { return m_pOutlierConfig; }
			const auto& getClusterConfig(bool vIsLitter) const { return vIsLitter ? m_BackgroundMarker.getClusterConfig() : m_LitterMarker.getClusterConfig(); }
			const auto& getClusterSet() const { return m_PointClusterSet; }
			const auto& getLabelSet() const { return m_PointLabelSet; }
			const auto& getLitterMarker() const { return m_LitterMarker; }
			const auto& getBackgroundMarker() const { return m_BackgroundMarker; }
			const auto& getNeighborhoodBuilder() const { return m_pNeighborhoodBuilder; }
			const auto& getStatusQueue() const { return m_StatusQueue; }
			const auto& getConfig() const { return m_pConfig; }
			CPointCluster* generateInitialCluster(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix, const std::function<double(Eigen::Vector2d)>& vHardnessFunc, EPointLabel vTargetLabel)
			{
				return __generateInitialCluster(vUserMarkedRegion, vPvMatrix, vHardnessFunc, vTargetLabel);
			}
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
			CPrecomputeManager		*m_pPrecomputeManager = nullptr;
			INeighborhoodBuilder    *m_pNeighborhoodBuilder = nullptr;
			const hiveConfig::CHiveConfig* m_pConfig = nullptr;
			const hiveConfig::CHiveConfig* m_pOutlierConfig = nullptr;
			
			std::deque<std::pair<CPointLabelSet, std::uint32_t>> m_StatusQueue;
			
			CPointCluster* __generateInitialCluster(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix, const std::function<double(Eigen::Vector2d)>& vHardnessFunc, EPointLabel vTargetLabel);

			bool __reset();

		friend class hiveDesignPattern::CSingleton<CPointCloudRetouchManager>;
		};
	}
}