#pragma once
#include "pch.h"

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		class CFindSplitPlane
		{
		public:
			CFindSplitPlane() = default;
			~CFindSplitPlane() = default;

			Eigen::Vector4f execute(pcl::PointCloud<pcl::PointXYZ>::Ptr vCloudOne, pcl::PointCloud<pcl::PointXYZ>::Ptr vCloudTwo);
			
			void __getMinAndMaxPointOfAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr vCloud,pcl::PointXYZ& vMinPoint, pcl::PointXYZ& vMaxPoint);
			void __judgeSplitPlane(int vAxisFlag,float vMinAxisValueCloudOne, float vMaxAxisValueCloudOne, float vMinAxisValueCloudTwo, float vMaxAxisValueCloudTwo, Eigen::Vector4f& vioSplitPlane);
		};
	}
}