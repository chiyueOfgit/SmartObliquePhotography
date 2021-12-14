#pragma once
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

namespace hiveObliquePhotography
{
	class CPCLPointCloudWrapper
	{
	public:
		CPCLPointCloudWrapper();
		~CPCLPointCloudWrapper();

		[[nodiscard]] bool init(std::uint8_t vPointTypeFlag);

		template<class T>
		pcl::PointCloud<T>* getPointCloud() const
		{
			
		}

	private:
		std::uint8_t m_PointTypeFlag = 0;
		pcl::PointCloud<pcl::PointXYZ>	        *m_pPointCloudXYZ = nullptr;
		pcl::PointCloud<pcl::PointXYZRGB>       *m_pPointCloudXYZRGB = nullptr;
		pcl::PointCloud<pcl::PointXYZRGBNormal> *m_pPointCloudXYZRGBNormal = nullptr;
	};
}

