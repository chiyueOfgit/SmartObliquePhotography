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

			virtual void runV(pcl::Indices& voObjectIndices,const Eigen::Vector2i& vResolution);

#ifdef _UNIT_TEST
			CImage<std::array<int, 1>> generateElevationMap(Eigen::Vector2i& vResolution) { return __generateElevationMap(vResolution); }
#endif
		
		private: 
			CImage<std::array<int, 1>> __generateElevationMap(const Eigen::Vector2i& vResolution);
			void __extractObjectIndices(const CImage<std::array<int, 1>>& vElevationMap, pcl::Indices& voIndices);

			void __calcAreaElevation(const Eigen::Vector2f& vMinCoord, const Eigen::Vector2f& vOffset, std::vector<std::vector<float>>& vioHeightSet);
			std::array<int, 1> __transElevation2Color(float vElevation, float vHeightDiff);
		};
	}
}
