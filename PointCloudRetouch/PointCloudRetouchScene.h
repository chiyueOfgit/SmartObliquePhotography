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
			void dumpPoint(pcl::index_t vIndex, Point_t& voPoint) const
			{
				_ASSERTE(vIndex < m_pPointCloudScene->size());
				__dumpPoint(m_pPointCloudScene->points[vIndex], voPoint);
			}
			
			template<typename Point_t>
			void dumpPointCloud(pcl::PointCloud<Point_t>& voPointCloud) const
			{
				for (const auto& Point : *m_pPointCloudScene)
				{
					Point_t TempPoint;
					__dumpPoint(Point, TempPoint);
					voPointCloud.push_back(TempPoint);
				}
			}
			
			template<typename Point_t>
			void dumpPointCloud(const std::vector<pcl::index_t>& vIndices, pcl::PointCloud<Point_t>& voPointCloud) const
			{
				for (auto Index : vIndices)
				{
					_ASSERTE(Index < m_pPointCloudScene->size());
					Point_t TempPoint;
					__dumpPoint(m_pPointCloudScene->points[Index], TempPoint);
					voPointCloud.push_back(TempPoint);
				}
			}

		private:
			Eigen::Vector3i __extractRgba(float vRgba) const;

			template<typename Point_t>
			void __dumpPoint(const PointCloud_t::PointType& vSrc, Point_t& voDst) const
			{
				if constexpr (pcl::traits::has_xyz_v<Point_t>)
					memcpy(voDst.data, vSrc.data, sizeof(vSrc.data));
				if constexpr (pcl::traits::has_normal_v<Point_t>)
					memcpy(voDst.data_n, vSrc.data_n, sizeof(vSrc.data_n));
				if constexpr (pcl::traits::has_color_v<Point_t>)
					voDst.rgba = vSrc.rgba;
			}

			PointCloud_t::Ptr m_pPointCloudScene = nullptr;
		};
	}
}


