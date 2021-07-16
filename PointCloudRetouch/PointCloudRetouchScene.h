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

			template<typename Point_t>
			void dumpPointCloud(const std::vector<pcl::index_t>& vIndices, pcl::PointCloud<Point_t>& voPointCloud) const
			{
				for (auto Index : vIndices)
				{
					_ASSERTE(Index < m_pPointCloudScene->size());
					Point_t Point;

					if constexpr(pcl::traits::has_xyz_v<Point_t>)
						memcpy(Point.data, m_pPointCloudScene->points[Index].data, sizeof(Point.data));
					if constexpr (pcl::traits::has_normal_v<Point_t>)
						memcpy(Point.data_n, m_pPointCloudScene->points[Index].data_n, sizeof(Point.data_n));
					if constexpr (pcl::traits::has_color_v<Point_t>)
						Point.rgba = m_pPointCloudScene->points[Index].rgba;

					voPointCloud.push_back(Point);
				}
			}

		private:
			Eigen::Vector3i __extractRgba(float vRgba) const;

			PointCloud_t::Ptr m_pPointCloudScene = nullptr;
		};
	}
}


