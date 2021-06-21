#pragma once
#include "PointCloudRetouchScene.h"
#include "PointClusterSet.h"
#include "PointLabelSet.h"
#include "RetouchTask.h"
#include "InitialClusterCreator.h"

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

			void buildNeighborhood(pcl::index_t vSeed, std::uint32_t vSeedClusterIndex, std::vector<pcl::index_t>& voNeighborhood);
			void tagPointLabel(pcl::index_t vPoint, EPointLabel vTargetLabel, std::uint32_t vClusterIndex, double vClusterBelongingProbability)
			{
				m_PointLabelSet.tagPointLabel(vPoint, vTargetLabel, vClusterIndex, vClusterBelongingProbability);
			}

			bool init(PointCloud_t::Ptr vPointCloud, const hiveConfig::CHiveConfig* vConfig);
			bool executeMarker(const std::vector<pcl::index_t>& vUserMarkedRegion, double vHardness, double vRadius, const Eigen::Vector3f& vCameraPos, const Eigen::Matrix4d& vPvMatrix, EPointLabel vTargetLabel);

			std::size_t   getNumCluster() const { return m_PointClusterSet.getNumCluster(); }
			std::uint32_t addAndGetTimestamp() { m_Timestamp++; return m_Timestamp; }
			std::uint32_t getClusterIndexAt(std::size_t vIndex) const { return m_PointLabelSet.getClusterIndexAt(vIndex); }

			double getClusterBelongingProbabilityAt(std::size_t vIndex) const { return m_PointLabelSet.getClusterBelongingProbabilityAt(vIndex); }

			const auto& getRetouchScene() const { return m_Scene; }

#ifdef _UNIT_TEST
			const auto& getClusterSet() const { return m_PointClusterSet; }
			const auto& getLabelSet() const { return m_PointLabelSet; }
			const auto& getRetouchScene() const { return m_Scene; }
			const auto& getLitterMarker() const { return m_LitterMarker; }
			const auto& getBackgroundMarker() const { return m_BackgroundMarker; }
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
			INeighborhoodBuilder    *m_pNeighborhoodBuilder = nullptr;
			hiveConfig::CHiveConfig* m_pConfig;
			
			CPointCluster* __generateInitialCluster(const std::vector<pcl::index_t>& vUserMarkedRegion, double vHardness, double vRadius, const Eigen::Vector3f& vCameraPos, const Eigen::Matrix4d& vPvMatrix, EPointLabel vTargetLabel);

		friend class hiveDesignPattern::CSingleton<CPointCloudRetouchManager>;
		};
	}
}