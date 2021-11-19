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
			void map2Cloud(const CImage<float>& vTexture, std::vector<pcl::index_t>& voCandidates, bool vIfObject) { return __map2Cloud(vTexture, voCandidates, vIfObject); }
			void map2Cloud(const CImage<float>& vTexture, std::vector<std::vector<pcl::index_t>>& voEdgeIndices, const std::vector<std::vector<Eigen::Vector2i>>& vEdgeSet)
			{  __map2Cloud(vTexture, voEdgeIndices, vEdgeSet); }
			//CImage<float> generateElevationMap(Eigen::Vector2i& vResolution) { return __generateElevationMap(vResolution); }
			CImage<float> generateGrownImage(const CImage<float>& vElevationMap)
			{
				auto ExtractedImage = __generateMaskByGrowing(vElevationMap, 5);
				__extractObjectByMask(vElevationMap, ExtractedImage);
				return ExtractedImage;
			}
			CImage<float> generateEdgeImage(const CImage<float>& vGrownImage) { return __extractGroundEdgeImage(vGrownImage); }
			std::vector<std::vector<Eigen::Vector2i>> divide2EdgeSet(const CImage<float>& vEdgeImage) { return __divide2EdgeSet(vEdgeImage); };
#endif
		
		private: 
			std::vector<std::vector<std::vector<pcl::index_t>>> m_PointDistributionSet;

			void __extractObjectIndices(const CImage<float>& vElevationMap, pcl::Indices& voIndices, std::vector<std::vector<pcl::index_t>>& voEdgeIndices);
			
			Eigen::Vector2i __findStartPoint(const CImage<float>& vImage);
			CImage<float> __generateMaskByGrowing(const CImage<float>& vOriginImage, int vThreshold);
			void __extractObjectByMask(const CImage<float>& vOriginImage, CImage<float>& vioMaskImage);
			CImage<float> __extractGroundEdgeImage(const CImage<float>& vExtractedImage);
				
			void __map2Cloud(const CImage<float>& vTexture, std::vector<pcl::index_t>& voCandidates, bool vIfObject);
			void __map2Cloud(const CImage<float>& vTexture, std::vector<std::vector<pcl::index_t>>& voEdgeIndices, const std::vector<std::vector<Eigen::Vector2i>>& vEdgeSet);
			
			bool __findBlackPoint(const CImage<float>& vImage, Eigen::Vector2i& voBlackPoint);
			std::vector<std::vector<Eigen::Vector2i>> __divide2EdgeSet(const CImage<float>& vEdgeImage);
			
		};
	}
}
