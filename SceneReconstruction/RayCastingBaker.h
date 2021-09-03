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
			bool __isPointInTriangle(const Eigen::Vector2f& vPoint, const Eigen::Vector2f& vA, const Eigen::Vector2f& vB, const Eigen::Vector2f& vC);
			Eigen::Vector3f __calcBarycentric(const Eigen::Vector2f& vPoint, const Eigen::Vector2f& vA, const Eigen::Vector2f& vB, const Eigen::Vector2f& vC);
			Eigen::Vector3f __calcRayDirection(const STexelInfo& vInfo);
			std::vector<pcl::index_t> __cullPointsByRay(const Eigen::Vector3f& vRayBegin, const Eigen::Vector3f& vRayDirection);

			std::tuple<float, float, float> __calcBarycentricCoord(const Eigen::Vector2f& vPointA, const Eigen::Vector2f& vPointB, const Eigen::Vector2f& vPointC, const Eigen::Vector2f& vPointP);
			std::pair<Eigen::Vector2f, Eigen::Vector2f> __calcBoxInTextureCoord(const Eigen::Vector2f& vPointA, const Eigen::Vector2f& vPointB, const Eigen::Vector2f& vPointC);
			Eigen::Vector3f __calcTexelCoordInWorld(const Eigen::Vector3f& vPointA, const Eigen::Vector3f& vPointB, const Eigen::Vector3f& vPointC, const std::tuple<float, float, float>& vBarycentricCoord);

			PointCloud_t::Ptr m_pCloud = nullptr;
		};
	}
}