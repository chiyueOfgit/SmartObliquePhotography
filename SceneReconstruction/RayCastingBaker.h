#pragma once
#include "TextureBaker.h"
#include <flann/flann.hpp>

namespace hiveObliquePhotography
{
	namespace SceneReconstruction
	{
		struct SRay
		{
			Eigen::Vector3f Origin;
			Eigen::Vector3f Direction;
		};
		
		struct STexelInfo
		{
			Eigen::Vector2i TexelCoord;
			std::vector<SRay> RaySet;
		};

		struct SCandidateInfo
		{
			Eigen::Vector3f Intersection;
			pcl::index_t SurfelIndex;
		};

		class CRayCastingBaker : public ITextureBaker
		{
		public:
			CRayCastingBaker() = default;
			~CRayCastingBaker() = default;

			CImage<std::array<int, 3>> bakeTexture(PointCloud_t::Ptr vPointCloud, const Eigen::Vector2i& vResolution) override;
			std::vector<STexelInfo> findSamplesPerFace(const SFace& vFace, const Eigen::Vector2i& vResolution) const;
			std::vector<SCandidateInfo> executeIntersection(const SRay& vRay) const;
			std::array<int, 3> calcTexelColor(const std::vector<SCandidateInfo>& vCandidates, const SRay& vRay) const;
			
#ifdef _UNIT_TEST
			void setPointCloud(PointCloud_t::Ptr vCloud) { m_pCloud = vCloud; __buildKdTree(m_pCloud); }
#endif // _UNIT_TEST

		private:
			void __buildKdTree(PointCloud_t::Ptr vCloud);
			std::pair<Eigen::Vector2f, Eigen::Vector2f> __calcBoxInTextureCoord(const Eigen::Vector2f& vPointA, const Eigen::Vector2f& vPointB, const Eigen::Vector2f& vPointC) const;
			Eigen::Vector3f __calcBarycentricCoord(const Eigen::Vector2f& vPointA, const Eigen::Vector2f& vPointB, const Eigen::Vector2f& vPointC, const Eigen::Vector2f& vPointP) const;
			SRay __calcRay(const SFace& vFace, const Eigen::Vector3f& vBarycentricCoord) const;
			std::vector<pcl::index_t> __cullPointsByRay(const Eigen::Vector3f& vRayOrigin, const Eigen::Vector3f& vRayDirection) const;

			PointCloud_t::Ptr m_pCloud = nullptr;

			std::pair<flann::Index<flann::L2<float>>*, Eigen::Matrix<float, -1, -1, Eigen::RowMajor>> m_KdTree;
		};
	}
}