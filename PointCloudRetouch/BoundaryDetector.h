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

			virtual void runV(std::vector<pcl::index_t>& vBoundarySet, std::vector<std::vector<pcl::index_t>>& voHoleSet, const hiveConfig::CHiveConfig* vConfig);

		private:
			Eigen::Vector3f __calcProjectivePoint(Eigen::Vector3f& vCenterPosition, Eigen::Vector3f& vCenterNormal, Eigen::Vector3f& vProjectPosition);
			float __calcAngle(Eigen::Vector3f& vStandardVector, Eigen::Vector3f& vOtherVector, Eigen::Vector3f& vCenterNormal);
			void __calcFitPlaneLeastSquares(Eigen::Vector3f& voPlaneCoeff, const std::vector<Eigen::Vector3f>& vData);
			Eigen::Vector3f __calcFitPlaneRANSAC(const std::vector<Eigen::Vector3f>& vData, double vDistanceThreshold, const Eigen::Vector3f& vUp);
			void __divideBoundary(std::vector<pcl::index_t>& vBoundaryPointSet, std::vector<std::vector<pcl::index_t>>& voHoleSet);
			void __getNearestNeighbor(pcl::index_t vSeed, std::vector<pcl::index_t>& vTotalSet, std::vector<pcl::index_t>& voNeighborSet);
		};
	}
}
