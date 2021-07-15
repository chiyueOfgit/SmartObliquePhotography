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

		private:
			Eigen::Vector3i __extractRgba(float vRgba) const;

			PointCloud_t::Ptr m_pPointCloudScene = nullptr;
		};
	}
}


