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
			CImage<std::array<int, 3>> generateElevationMap(Eigen::Vector2i& vResolution) { return __generateElevationMap(vResolution); }
			void map2Cloud(const CImage<std::array<int, 1>>& vTexture, std::vector<pcl::index_t>& voCandidates) { return __map2Cloud(vTexture, voCandidates); }
#endif
		
		private: 
			CImage<std::array<int, 3>> __generateElevationMap(const Eigen::Vector2i& vResolution);
			void __extractObjectIndices(const CImage<std::array<int, 3>>& vElevationMap, pcl::Indices& voIndices);

			std::array<int, 3> __transElevation2Color(float vElevation, float vHeightDiff);
			void __calcAreaElevation(const Eigen::Vector2f& vMinCoord, const Eigen::Vector2f& vOffset, std::vector<std::vector<float>>& vioHeightSet);

			void __map2Cloud(const CImage<std::array<int, 1>>& vTexture, std::vector<pcl::index_t>& voCandidates);

			std::pair<Eigen::Vector3f, Eigen::Vector3f> m_Box;
		};
	}
}
