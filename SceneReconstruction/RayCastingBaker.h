#pragma once
#include "TextureBaker.h"
#include <flann/flann.hpp>

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		struct STexelInfo
		{
			Eigen::Vector2i TexelPos;
			Eigen::Vector3f TexelPosInWorld;
			SFace OriginFace;
		};

		struct SCandidateInfo
		{
			Eigen::Vector3f Pos;
			pcl::index_t PointIndex;
		};

		class CRayCastingBaker : public ITextureBaker
		{
		public:
			CRayCastingBaker() = default;
			~CRayCastingBaker() = default;

			virtual CImage<Eigen::Vector3i> bakeTexture(PointCloud_t::Ptr vPointCloud, const Eigen::Vector2i& vResolution) override;
			std::vector<STexelInfo> findTexelsPerFace(const SFace& vFace, Eigen::Vector2i vResolution);
			std::vector<SCandidateInfo> executeIntersection(const STexelInfo& vInfo);
			Eigen::Vector3i calcTexelColor(const std::vector<SCandidateInfo>& vCandidates, const STexelInfo& vInfo);
		    
#ifdef _UNIT_TEST
			void setPointCloud(PointCloud_t::Ptr vCloud) { m_pCloud = vCloud; __buildKdTree(m_pCloud); }
#endif // _UNIT_TEST

		private:
			void __buildKdTree(PointCloud_t::Ptr vCloud);
			std::pair<Eigen::Vector2f, Eigen::Vector2f> __calcBoxInTextureCoord(const Eigen::Vector2f& vPointA, const Eigen::Vector2f& vPointB, const Eigen::Vector2f& vPointC);
			Eigen::Vector3f __calcBarycentricCoord(const Eigen::Vector2f& vPointA, const Eigen::Vector2f& vPointB, const Eigen::Vector2f& vPointC, const Eigen::Vector2f& vPointP);
			Eigen::Vector3f __calcTexelPosInWorld(const Eigen::Vector3f& vPointA, const Eigen::Vector3f& vPointB, const Eigen::Vector3f& vPointC, const Eigen::Vector3f& vBarycentricCoord);
			Eigen::Vector3f __calcRayDirection(const STexelInfo& vInfo);
			std::vector<pcl::index_t> __cullPointsByRay(const Eigen::Vector3f& vRayOrigin, const Eigen::Vector3f& vRayDirection);

			PointCloud_t::Ptr m_pCloud = nullptr;

			std::pair<flann::Index<flann::L2<float>>*, Eigen::Matrix<float, -1, -1, Eigen::RowMajor>> m_KdTree;
		};
	}
}