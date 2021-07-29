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

			virtual void runV(std::vector<pcl::index_t>& vioBoundarySet, const hiveConfig::CHiveConfig* vConfig);	//vio: 非空传进指定候选点集，空传进为整个场景

		private:
			Eigen::Vector3f __calcProjectivePoint(Eigen::Vector3f& vCenterPosition, Eigen::Vector3f& vCenterNormal, Eigen::Vector3f& vProjectPosition);
			float __calcAngle(Eigen::Vector3f& vStandardVector, Eigen::Vector3f& vOtherVector, Eigen::Vector3f& vCenterNormal);
			void __calcFitPlane(Eigen::Vector3f& voPlaneCoeff, const std::vector<Eigen::Vector3f>& vData);
			Eigen::Vector3f fitPlane(const std::vector<Eigen::Vector3f>& vData, double vDistanceThreshold, const Eigen::Vector3f& vUp);
		};
	}
}
