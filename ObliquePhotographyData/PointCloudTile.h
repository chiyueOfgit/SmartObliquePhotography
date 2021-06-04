#pragma once
#include "pch.h"

namespace hiveObliquePhotography
{
	class CPointCloudTile
	{
	public:
		CPointCloudTile() = delete;
		CPointCloudTile(pcl::PointCloud<pcl::PointSurfel>* vPointCloud) : m_pPointCloud(vPointCloud) { _ASSERTE(m_pPointCloud); }
		~CPointCloudTile();

		void clear();

		std::size_t getSize() const { _ASSERTE(m_pPointCloud); return m_pPointCloud->size(); }

	private:
		std::string m_FileName;
		pcl::PointCloud<pcl::PointSurfel> *m_pPointCloud=nullptr;
	};
}

