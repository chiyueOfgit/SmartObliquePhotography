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

			virtual void runV(pcl::Indices& voObjectIndices, std::vector<std::vector<pcl::index_t>>& voEdgeIndices, const Eigen::Vector2i& vResolution);

#ifdef _UNIT_TEST
			void map2Cloud(const CImage<std::array<int, 1>>& vTexture, std::vector<pcl::index_t>& voCandidates, bool vIfObject) { return __map2Cloud(vTexture, voCandidates, vIfObject); }
			void map2Cloud(const CImage<std::array<int, 1>>& vTexture, std::vector<std::vector<pcl::index_t>>& voEdgeIndices, const std::vector<std::vector<Eigen::Vector2i>>& vEdgeSet)
			{  __map2Cloud(vTexture, voEdgeIndices, vEdgeSet); }
			CImage<std::array<int, 1>> generateElevationMap(Eigen::Vector2i& vResolution) { return __generateElevationMap(vResolution); }
			CImage<std::array<int, 1>> generateGrownImage(const CImage<std::array<int, 1>>& vElevationMap)
			{
				auto ExtractedImage = __generateMaskByGrowing(vElevationMap, 5);
				__extractObjectByMask(vElevationMap, ExtractedImage);
				return ExtractedImage;
			}
			CImage<std::array<int, 1>> generateEdgeImage(const CImage<std::array<int, 1>>& vGrownImage) { return __extractGroundEdgeImage(vGrownImage); }
			std::vector<std::vector<Eigen::Vector2i>> divide2EdgeSet(const CImage<std::array<int, 1>>& vEdgeImage) { return __divide2EdgeSet(vEdgeImage); };
#endif
		
		private: 
			std::vector<std::vector<std::vector<pcl::index_t>>> m_PointDistributionSet;

			CImage<std::array<int, 1>> __generateElevationMap(const Eigen::Vector2i& vResolution);
			void __extractObjectIndices(const CImage<std::array<int, 1>>& vElevationMap, pcl::Indices& voIndices, std::vector<std::vector<pcl::index_t>>& voEdgeIndices);

			std::array<int, 1> __transElevation2Color(float vElevation, float vHeightDiff);
			void __calcAreaElevation(const Eigen::Vector2f& vMinCoord, const Eigen::Vector2f& vOffset, std::vector<std::vector<float>>& vioHeightSet);
			
			Eigen::Vector2i __findStartPoint(const CImage<std::array<int, 1>>& vImage);
			CImage<std::array<int, 1>> __generateMaskByGrowing(const CImage<std::array<int, 1>>& vOriginImage, int vThreshold);
			void __extractObjectByMask(const CImage<std::array<int, 1>>& vOriginImage, CImage<std::array<int, 1>>& vioMaskImage);
			CImage<std::array<int, 1>> __extractGroundEdgeImage(const CImage<std::array<int, 1>>& vExtractedImage);
				
			void __map2Cloud(const CImage<std::array<int, 1>>& vTexture, std::vector<pcl::index_t>& voCandidates, bool vIfObject);
			void __map2Cloud(const CImage<std::array<int, 1>>& vTexture, std::vector<std::vector<pcl::index_t>>& voEdgeIndices, const std::vector<std::vector<Eigen::Vector2i>>& vEdgeSet);
			
			bool __findBlackPoint(const CImage<std::array<int, 1>>& vImage, Eigen::Vector2i& voBlackPoint);
			std::vector<std::vector<Eigen::Vector2i>> __divide2EdgeSet(const CImage<std::array<int, 1>>& vEdgeImage);
			
		};
	}
}
