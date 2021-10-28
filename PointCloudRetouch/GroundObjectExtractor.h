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
			CImage<std::array<int, 3>> generateElevationMap(Eigen::Vector2i& vResolution) { return __generateElevationMap(vResolution); }
#endif
		
		private: 
			CImage<std::array<int, 3>> __generateElevationMap(Eigen::Vector2i& vResolution);
			void __extractObjectIndices(CImage<std::array<int, 3>>& vElevationMap, pcl::Indices& voIndices);

			float __calcAreaElevation(Eigen::Vector2f& vMinCoord, Eigen::Vector2f& vOffset);
			std::array<int, 3> __transElevation2Color(float vElevation, float vMinElevation);
		};
	}
}
