#pragma once
#include <pcl/common/centroid.h>

namespace hiveObliquePhotography
{
	namespace PointCloudRetouch
	{
		using PointCloud_t = pcl::PointCloud<pcl::PointXYZRGBNormal>;

		namespace KEYWORD
		{
			const std::string PLANARITY_FEATURE = "PLANARITY_FEATURE";
			const std::string VFH_FEATURE = "VFH_FEATURE";
			const std::string COLOR_FEATURE = "COLOR_FEATURE";
			const std::string NORMAL_COMPLEXITY = "NORMAL_COMPLEXITY";

			const std::string EUCLIDEAN_NEIGHBOR_BUILDER = "EUCLIDEAN_NEIGHBOR_BUILDER";

			const std::string CLUSTER_EXPANDER = "CLUSTER_EXPANDER";
			const std::string CLUSTER_EXPANDER_MULTITHREAD = "CLUSTER_EXPANDER_MULTITHREAD";
			const std::string OUTLIER_DETECTOR = "OUTLIER_DETECTOR";
			const std::string BOUNDARY_DETECTOR = "BOUNDARY_DETECTOR";
			const std::string HOLE_REPAIRER = "HOLE_REPAIRER";

			const std::string TEMP_FOLDER = "Temp/";

		}
		enum class EPointLabel : unsigned char
		{
			DISCARDED,
			KEPT,
			UNWANTED,
			UNDETERMINED,
			FILLED,
		};

		template <typename T>
		T NormalDistribution(T vX, T vDelta  = 1)
		{
			return exp(-(vX * vX) / (2 * vDelta * vDelta)) / (2.50662827464 * vDelta);
		}

		static std::tuple<Eigen::Matrix3f, Eigen::Vector3f, Eigen::Vector3f> calcOBB(std::vector<Eigen::Vector3f>& vInput)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud(new pcl::PointCloud<pcl::PointXYZ>);
			for (auto Pos : vInput)
			{
				pcl::PointXYZ TempPoint;
				TempPoint.x = Pos.x(); TempPoint.y = Pos.y(); TempPoint.z = Pos.z();
				pCloud->push_back(TempPoint);
			}
			Eigen::Matrix3f CovarianceMatrix;
			Eigen::Vector4f Centroid;
			pcl::compute3DCentroid(*pCloud, Centroid);
			pcl::computeCovarianceMatrix(*pCloud, Centroid, CovarianceMatrix);
			CovarianceMatrix = CovarianceMatrix / pCloud->size();
			
			Eigen::EigenSolver<Eigen::Matrix3f> EigenMat(CovarianceMatrix);
			Eigen::Vector3f EigenValue = EigenMat.pseudoEigenvalueMatrix().diagonal();
			Eigen::Matrix3f EigenVector = EigenMat.pseudoEigenvectors();

			std::vector<std::tuple<float, Eigen::Vector3f>> EigenValueAndVector;
			int Size = static_cast<int>(EigenValue.size());
			EigenValueAndVector.reserve(Size);

			if (EigenVector.col(0).cross(EigenVector.col(1)).normalized().dot(EigenVector.col(2)) < 0)
				EigenVector.col(2).swap(-EigenVector.col(2));
			
			for (int i = 0; i < Size; ++i)
				EigenValueAndVector.push_back(std::tuple<float, Eigen::Vector3f>(abs(EigenValue[i]), EigenVector.col(i)));
			std::ranges::sort(EigenValueAndVector,
				[&](const std::tuple<float, Eigen::Vector3f>& a, const std::tuple<float, Eigen::Vector3f>& b) -> bool {
					return std::get<0>(a) > std::get<0>(b);
				});
			for (int i = 0; i < Size; ++i)
			{
				//EigenVector.col(i).swap(std::get<1>(EigenValueAndVector[i]));
				for (int k = 0; k < Size; k++)
					EigenVector(i, k) = std::get<1>(EigenValueAndVector[i])[k];
			}

			Eigen::Vector3f Min{ FLT_MAX, FLT_MAX, FLT_MAX };
			Eigen::Vector3f Max{ -FLT_MAX, -FLT_MAX, -FLT_MAX };
			auto update = [&](const Eigen::Vector3f& vPos)
			{
				for (int i = 0; i < 3; i++)
				{
					if (vPos.data()[i] < Min.data()[i])
						Min.data()[i] = vPos.data()[i];
					if (vPos.data()[i] > Max.data()[i])
						Max.data()[i] = vPos.data()[i];
				}
			};

			for (auto& Pos : vInput)
			{
				Eigen::Vector3f AfterPos = EigenVector * Pos;
				update(AfterPos);
			}

			std::tuple<Eigen::Matrix3f, Eigen::Vector3f, Eigen::Vector3f> ObbBox(EigenVector, Min, Max);
			return ObbBox;
		}
	}
}
