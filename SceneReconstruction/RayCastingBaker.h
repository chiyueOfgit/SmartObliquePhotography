#pragma once
#include "TextureBaker.h"

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

			virtual CImage<Eigen::Vector3i> bakeTexture(PointCloud_t::Ptr vPointCloud) override;
			std::vector<STexelInfo> findTexelsPerFace(const SFace& vFace, Eigen::Vector2i vResolution);
			std::vector<SCandidateInfo> executeIntersection(const STexelInfo& vInfo);
			Eigen::Vector3i calcTexelColor(const std::vector<SCandidateInfo>& vCandidates, const STexelInfo& vInfo);
		    
#ifdef _UNIT_TEST
			void setPointCloud(PointCloud_t::Ptr vCloud) { m_pCloud = vCloud; }
#endif // _UNIT_TEST

		private:
			Eigen::Vector3f __calcRayDirection(const STexelInfo& vInfo);
			std::vector<pcl::index_t> __cullPointsByRay(const Eigen::Vector3f& vRayBegin, const Eigen::Vector3f& vRayDirection);

			PointCloud_t::Ptr m_pCloud = nullptr;
		};
	}
}