#pragma once
#include "PointClusterSet.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class INeighborhoodBuilder;

		class CPointCloudRetouchScene 
		{
		public:
			CPointCloudRetouchScene();
			~CPointCloudRetouchScene();

			void init(PointCloud_t::Ptr vPointCloudScene);
			void reset() { m_pPointCloudScene = nullptr; }

			Eigen::Vector4f getPositionAt(pcl::index_t vIndex) const;
			Eigen::Vector4f getNormalAt(pcl::index_t vIndex) const;
			Eigen::Vector3i getColorAt(pcl::index_t vIndex) const;
			
			std::size_t getNumPoint() const { return m_pPointCloudScene ? m_pPointCloudScene->size() : 0; }

			std::pair<Eigen::Vector3f, Eigen::Vector3f> getBoundingBox(const std::vector<pcl::index_t>& vIndices) const;

			template<typename Point_t>
			void dumpPoint(pcl::index_t vIndex, Point_t& voPoint) const
			{
				_ASSERTE(vIndex < m_pPointCloudScene->size());
				pcl::copyPoint(m_pPointCloudScene->at(vIndex), voPoint);
			}
			
			template<typename Point_t>
			void dumpPointCloud(pcl::PointCloud<Point_t>& voPointCloud) const
			{
				pcl::copyPointCloud(*m_pPointCloudScene, voPointCloud);
			}
			
			template<typename Point_t>
			void dumpPointCloud(const std::vector<pcl::index_t>& vIndices, pcl::PointCloud<Point_t>& voPointCloud) const
			{
				pcl::copyPointCloud(*m_pPointCloudScene, vIndices, voPointCloud);
			}

		private:
			Eigen::Vector3i __extractRgba(float vRgba) const;

			PointCloud_t::Ptr m_pPointCloudScene = nullptr;
		};
	}
}


