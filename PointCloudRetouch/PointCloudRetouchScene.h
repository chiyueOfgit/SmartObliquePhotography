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
			void reset(std::uint32_t vTimestamp) {};

			Eigen::Vector4d getPositionAt(pcl::index_t vIndex);
			Eigen::Vector4d getNormalAt(pcl::index_t vIndex);
			Eigen::Vector4i getColorAt(pcl::index_t vIndex);

			std::size_t getNumPoint() const { _ASSERTE(m_pPointCloudScene); return m_pPointCloudScene->size(); }

		private:
			Eigen::Vector4i __extractRgba(float vRgba);

			PointCloud_t::Ptr m_pPointCloudScene = nullptr;
		};
	}
}


