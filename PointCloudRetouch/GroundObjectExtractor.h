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
			void map2Cloud(const CImage<std::array<int, 1>>& vTexture, std::vector<pcl::index_t>& voCandidates) { return __map2Cloud(vTexture, voCandidates); }
			CImage<std::array<int, 1>> generateElevationMap(Eigen::Vector2i& vResolution) { return __generateElevationMap(vResolution); }
#endif
		
		private: 
			std::vector<std::vector<pcl::index_t>> m_PointDistributionSet;

			CImage<std::array<int, 1>> __generateElevationMap(const Eigen::Vector2i& vResolution);
			void __extractObjectIndices(const CImage<std::array<int, 1>>& vElevationMap, pcl::Indices& voIndices);

			std::array<int, 1> __transElevation2Color(float vElevation, float vHeightDiff);
			void __calcAreaElevation(const Eigen::Vector2f& vMinCoord, const Eigen::Vector2f& vOffset, std::vector<std::vector<float>>& vioHeightSet);
			
			Eigen::Vector2i __findStartPoint(const CImage<std::array<int, 1>>& vImage);
			CImage<std::array<int, 1>> __generateMaskByGrowing(const CImage<std::array<int, 1>>& vOriginImage, int vThreshold);
			void __extractObjectByMask(const CImage<std::array<int, 1>>& vOriginImage, CImage<std::array<int, 1>>& vioMaskImage);
			
			void __map2Cloud(const CImage<std::array<int, 1>>& vTexture, std::vector<pcl::index_t>& voCandidates);

		};
	}
}
