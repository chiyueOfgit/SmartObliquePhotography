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
		
		private:
			CImage<std::array<int, 3>> __generateElevationMap(Eigen::Vector2i& vResolution);
			void __extractObjectIndices(CImage<std::array<int, 3>>& vElevationMap, pcl::Indices& voIndices);


			
		};
	}
}
