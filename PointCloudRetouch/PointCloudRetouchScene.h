#pragma once
#include "PointClusterSet.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class INeighborhoodBuilder;

		class CPointCloudScene 
		{
		public:
			CPointCloudScene();
			~CPointCloudScene();

			void init(const std::vector<PointCloud_t::Ptr>& vTileSet);
			void reset() { m_TileSet.clear(); }

			Eigen::Vector4f getPositionAt(pcl::index_t vIndex) const;
			Eigen::Vector4f getNormalAt(pcl::index_t vIndex) const;
			Eigen::Vector3i getColorAt(pcl::index_t vIndex) const;
			
			std::size_t getTileIndexByPoint(pcl::index_t vIndex) const;

			std::size_t getTileOffset(std::size_t vTileIndex) const { return m_TileSet[vTileIndex].first; }
			std::size_t getTileNumPoints(std::size_t vTileIndex) const { return m_TileSet[vTileIndex].second->size(); }
			std::size_t getNumTile() const { return m_TileSet.size(); }

			std::size_t getNumPoint() const { return m_NumPoints; }

			std::pair<Eigen::Vector3f, Eigen::Vector3f> getBoundingBox(const std::vector<pcl::index_t>& vIndices) const;
			std::vector<pcl::index_t> getPointsInBox(const std::pair<Eigen::Vector3f, Eigen::Vector3f>& vBox, const Eigen::Matrix3f& vRotationMatrix) const;

			template<typename Point_t>
			void dumpPoint(pcl::index_t vIndex, Point_t& voPoint) const
			{
				pcl::copyPoint(__getPoint(vIndex), voPoint);
			}
			
			template<typename Point_t>
			void dumpPointCloud(pcl::PointCloud<Point_t>& voPointCloud) const
			{
				PointCloud_t TempCloud;
				for (auto Pair : m_TileSet)
					TempCloud += *Pair.second;
				pcl::copyPointCloud(TempCloud, voPointCloud);
			}
			
			template<typename Point_t>
			void dumpPointCloud(const std::vector<pcl::index_t>& vIndices, pcl::PointCloud<Point_t>& voPointCloud) const
			{
				PointCloud_t TempCloud;
				for (auto Index : vIndices)
					TempCloud.push_back(__getPoint(Index));
				pcl::copyPointCloud(TempCloud, voPointCloud);
			}

		private:
			inline PointCloud_t::PointType __getPoint(pcl::index_t vIndex) const;
			Eigen::Vector3i __extractRgba(float vRgba) const;

			std::vector<std::pair<int, PointCloud_t::Ptr>> m_TileSet;
			std::size_t m_NumPoints = 0;
		};
	}
}


