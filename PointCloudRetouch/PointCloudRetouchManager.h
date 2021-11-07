#pragma once
#include "PointCloudRetouchScene.h"
#include "PointClusterSet.h"
#include "PointLabelSet.h"
#include "RetouchTask.h"
#include "InitialClusterCreator.h"
#include "PointSetPreprocessor.h"
#include "HoleRepairer.h"
#include "GroundObjectExtractor.h"
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

			std::vector<pcl::index_t> buildNeighborhood(pcl::index_t vSeed, std::string& vType, float vPara);
			std::vector<pcl::index_t> buildNeighborhood(pcl::index_t vSeed);
			void tagPointLabel(pcl::index_t vPoint, EPointLabel vTargetLabel, std::uint32_t vClusterIndex, double vClusterBelongingProbability)
			{
				m_PointLabelSet.tagPointLabel(vPoint, vTargetLabel, vClusterIndex, vClusterBelongingProbability);
			}
			bool dumpPointLabel(std::vector<std::size_t>& voPointLabel) const;
			bool dumpTileLabel(std::size_t vTile, std::vector<std::size_t>& voTileLabel);
			bool dumpPointLabelAt(std::size_t& voPointLabel, std::uint32_t vIndex) const;

			void dumpExpandPoints(std::vector<int>& voExpandPoints, bool vIsLitterMarker) 
			{
				vIsLitterMarker ? m_LitterMarker.dumpTaskMarkedPoints(voExpandPoints) : m_BackgroundMarker.dumpTaskMarkedPoints(voExpandPoints);
			}

			bool init(const std::vector<PointCloud_t::Ptr>& vTileSet, const hiveConfig::CHiveConfig* vConfig);
			void clearMark();
			bool executePreprocessor(std::vector<pcl::index_t>& vioPointSet, const Eigen::Matrix4d& vPvMatrix, const std::function<double(Eigen::Vector2d)>& vSignedDistanceFunc, const Eigen::Vector3d& vViewPos);
			bool executeMarker(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix, const std::function<double(Eigen::Vector2d)>& vHardnessFunc, EPointLabel vTargetLabel);
			bool executeOutlierDetector(EPointLabel vTargetLabel);
			void executeHoleRepairerSetRegion(const std::vector<pcl::index_t>& vHoleRegion);
			void executeHoleRepairerSetInput(const std::vector<pcl::index_t>& vInput);
			void executeHoleRepairer(std::vector<pcl::PointXYZRGBNormal>& voNewPoints);
			void executeAutoMarker();
			void recordCurrentStatus();
			bool undo();
			void recoverMarkedPoints2Undetermined(EPointLabel vLabel);
			std::tuple<Eigen::Matrix3f, Eigen::Vector3f, Eigen::Vector3f> calcOBBByIndices(const std::vector<pcl::index_t>& vIndices);
			
			std::size_t   getNumCluster() const { return m_PointClusterSet.getNumCluster(); }
			std::uint32_t addAndGetTimestamp() { m_Timestamp++; return m_Timestamp; }
			std::uint32_t getClusterIndexAt(std::size_t vIndex) const { return m_PointLabelSet.getClusterIndexAt(vIndex); }

			double getClusterBelongingProbabilityAt(std::size_t vIndex) const { return m_PointLabelSet.getClusterBelongingProbabilityAt(vIndex); }
			
			void switchLabel(EPointLabel vTo, EPointLabel vFrom);
			void setLabel(const std::vector<pcl::index_t>& vPoints, EPointLabel vTarget);	//for perform
			void dumpIndicesByLabel(std::vector<pcl::index_t>& vioIndices, EPointLabel vLabel);
			EPointLabel getLabelAt(pcl::index_t vIndex) { return m_PointLabelSet.getLabelAt(vIndex); };

			bool dumpColorFeatureMainColors(std::vector<Eigen::Vector3i>& vMainColors) const;
			bool dumpColorFeatureNearestPoints(std::vector<pcl::index_t>& vNearestPoints) const;
			
			const auto& getScene() const { return m_Scene; }  //FIXME-014: 能不暴露m_Scene给外面吗？
			const auto& getPrecomputeManager() const { return m_pPrecomputeManager; }

#ifdef _UNIT_TEST
			void reset4UnitTest() { __reset(); }
			const auto& getOutlierConfig() const       { return m_pOutlierConfig; }
			const auto& getClusterSet() const		   { return m_PointClusterSet; }
			const auto& getLabelSet() const			   { return m_PointLabelSet; }
			const auto& getLitterMarker() const		   { return m_LitterMarker; }
			const auto& getBackgroundMarker() const    { return m_BackgroundMarker; }
			const auto& getNeighborhoodBuilder() const { return m_pNeighborhoodBuilder; }
			const auto& getStatusQueue() const		   { return m_StatusQueue; }
			const auto& getConfig() const			   { return m_pConfig; }
			const auto& getClusterConfig(bool vIsLitter) const { return vIsLitter ? m_BackgroundMarker.getClusterConfig() : m_LitterMarker.getClusterConfig(); }
			CPointCluster* generateInitialCluster(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix, const std::function<double(Eigen::Vector2d)>& vHardnessFunc, EPointLabel vTargetLabel)
			{
				return __generateInitialCluster(vUserMarkedRegion, vPvMatrix, vHardnessFunc, vTargetLabel);
			}
#endif // _UNIT_TEST


		private:
			CPointCloudRetouchManager() {};

			std::uint32_t m_Timestamp = 0;  //FIXME-014：这个变量实际有用吗？

			CPointClusterSet         m_PointClusterSet;
			CPointLabelSet           m_PointLabelSet;
			CPointCloudScene         m_Scene;
			CRetouchTask             m_LitterMarker;
			CRetouchTask             m_BackgroundMarker;
			CInitialClusterCreator   m_InitialClusterCreator;
			CPointSetPreprocessor    m_Preprocessor;
			CHoleRepairer			 m_HoleRepairer;
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