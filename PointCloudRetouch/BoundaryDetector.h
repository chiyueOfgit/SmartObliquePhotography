#pragma once
#include "PointClassifier.h"

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		class CBoundaryDetector : public IPointClassifier
		{
		public:
			CBoundaryDetector() = default;
			~CBoundaryDetector() = default;

			bool init(const hiveConfig::CHiveConfig* vConfig);
			virtual void runV(const std::vector<pcl::index_t>& vBoundarySet, std::vector<std::vector<pcl::index_t>>& voHoleSet);

		private:
			Eigen::Vector3f __calcProjectivePoint(Eigen::Vector3f& vCenterPosition, Eigen::Vector3f& vCenterNormal, Eigen::Vector3f& vProjectPosition);
			float __calcAngle(Eigen::Vector3f& vStandardVector, Eigen::Vector3f& vOtherVector, Eigen::Vector3f& vCenterNormal);
			void __divideBoundary(std::vector<pcl::index_t>& vBoundaryPointSet, std::vector<std::vector<pcl::index_t>>& voHoleSet);
			void __findNearestBoundaryPoint(pcl::index_t vSeed, std::vector<pcl::index_t>& vTotalSet, std::vector<pcl::index_t>& voNeighborSet, float vTolerance);
			bool __isClosedHoleBoundary(std::vector<pcl::index_t>& vHoleBoundary);
			const hiveConfig::CHiveConfig* m_pBoundaryDetectorConfig = nullptr;
		};
	}
}
