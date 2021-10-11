#pragma once

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		class CFindSplitPlane
		{
		public:
			CFindSplitPlane() = default;
			~CFindSplitPlane() = default;

			Eigen::Vector4f execute(PointCloud_t::Ptr vCloudOne, PointCloud_t::Ptr vCloudTwo);
			
			void __getMinAndMaxPointOfAABB(PointCloud_t::Ptr vCloud, PointCloud_t::PointType& voMinPoint, PointCloud_t::PointType& voMaxPoint);
			void __judgeSplitPlane(int vAxisFlag,float vMinAxisValueCloudOne, float vMaxAxisValueCloudOne, float vMinAxisValueCloudTwo, float vMaxAxisValueCloudTwo, Eigen::Vector4f& vioSplitPlane);
		};
	}
}