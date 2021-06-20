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

		class CPointCloudRetouchManager : public hiveDesignPattern::CSingleton<CPointCloudRetouchManager>
		{
		public:	
			~CPointCloudRetouchManager() = default;

			void tagPointLabel(const std::vector<pcl::index_t>& vTargetPointSet, EPointLabel vTargetLabel);

			bool init(PointCloud_t::Ptr vPointCloud, const hiveConfig::CHiveConfig* vConfig);
			bool executeMarker(const std::vector<pcl::index_t>& vUserMarkedRegion, double vHardness, double vRadius, const Eigen::Vector3f& vCameraPos, const Eigen::Matrix4d& vPvMatrix, EPointLabel vTargetLabel);

			std::size_t   getNumCluster() const { return m_PointClusterSet.getNumCluster(); }
			std::uint32_t addAndGetTimestamp() { m_Timestamp++; return m_Timestamp; }

		private:
			CPointCloudRetouchManager() {};

			std::uint32_t m_Timestamp = 0;

			CPointClusterSet         m_PointClusterSet;
			CPointLabelSet           m_PointLabelSet;
			CPointCloudRetouchScene  m_Scene;
			CRetouchTask             m_LitterMarker;
			CRetouchTask             m_BackgroundMarker;
			CInitialClusterCreator   m_InitialClusterCreator;
			hiveConfig::CHiveConfig* m_pConfig;
			
			CPointCluster* __generateInitialCluster(const std::vector<pcl::index_t>& vUserMarkedRegion, double vHardness, double vRadius, const Eigen::Vector3f& vCameraPos, const Eigen::Matrix4d& vPvMatrix, EPointLabel vTargetLabel);

		friend class hiveDesignPattern::CSingleton<CPointCloudRetouchManager>;
		};
	}
}