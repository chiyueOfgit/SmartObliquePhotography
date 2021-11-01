#pragma once
#include "PointClassifier.h"
#include "Image.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CGroundObjectExtractor : public IPointClassifier
		{
		public:
			CGroundObjectExtractor() = default;
			~CGroundObjectExtractor() = default;

			virtual void runV(pcl::Indices& voObjectIndices, Eigen::Vector2i& vResolution);

#ifdef _UNIT_TEST
			CImage<std::array<int, 1>> generateElevationMap(Eigen::Vector2i& vResolution) { return __generateElevationMap(vResolution); }
#endif
		
		private: 
			CImage<std::array<int, 1>> __generateElevationMap(Eigen::Vector2i& vResolution);
			void __extractObjectIndices(CImage<std::array<int, 1>>& vElevationMap, pcl::Indices& voIndices);

			void __calcAreaElevation(Eigen::Vector2f& vMinCoord, Eigen::Vector2f& vOffset, std::vector<std::vector<float>>& vioHeightSet);
			std::array<int, 3> __transElevation2Color(float vElevation, float vHeightDiff);
		};
	}
}
