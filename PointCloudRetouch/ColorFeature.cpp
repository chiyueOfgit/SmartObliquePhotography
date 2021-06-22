#include "pch.h"
#include "ColorFeature.h"
#include "PointCloudRetouchManager.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CColorFeature, KEYWORD::COLOR_FEATURE)

#define EPSILON 1e-2

//*****************************************************************
//FUNCTION: 
double CColorFeature::generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter)
{
	_ASSERTE(m_pConfig);
	{
		std::optional<float> ColorThreshold = m_pConfig->getAttribute<float>("COLOR_THRESHOLD");
		if (ColorThreshold.has_value())
			m_ColorThreshold = ColorThreshold.value();
	}
	{
		std::optional<int> NumMainColors = m_pConfig->getAttribute<int>("NUM_MAIN_COLORS");
		if (NumMainColors.has_value())
			m_NumMainColors = NumMainColors.value();
	}
	
	if (vDeterminantPointSet.empty())
		return 0.0;

	__computeMainColors(vDeterminantPointSet, m_MainBaseColors, m_NumMainColors);

	double Score = 0.0;
	for (auto ValidationIndex : vValidationSet)
	{
		Score += evaluateFeatureMatchFactorV(ValidationIndex);
	}
	return Score / vValidationSet.size();
}

//*****************************************************************
//FUNCTION: 
double CColorFeature::evaluateFeatureMatchFactorV(pcl::index_t vInputPoint)
{
	for (const auto& MainColor : m_MainBaseColors)
	{
		auto Color = CPointCloudRetouchManager::getInstance()->getRetouchScene().getColorAt(vInputPoint);

		if ((Color - MainColor).norm() < m_ColorThreshold)
			return 1.0;
	}
	return 0.0;
}

//*****************************************************************
//FUNCTION: 
void CColorFeature::__computeMainColors(const std::vector<pcl::index_t>& vPointIndices, std::vector<Eigen::Vector3i>& vMainColors, std::size_t vK)
{
	std::vector<Eigen::Vector3i> PointCloudColors;
	for (auto Index : vPointIndices)
		PointCloudColors.push_back(CPointCloudRetouchManager::getInstance()->getRetouchScene().getColorAt(Index));

	vMainColors = __kMeansCluster(PointCloudColors, vK);
}

//*****************************************************************
//FUNCTION: 
std::vector<Eigen::Vector3i> CColorFeature::__kMeansCluster(const std::vector<Eigen::Vector3i>& vData, std::size_t vK) const
{
	std::vector<int> PointTag4Cluster(vData.size(), -1);
	std::vector<Eigen::Vector3i> ClusterCentroids;
	double Variance = 0;

	for (int i = 0; i < vK; i++)
	{
		ClusterCentroids.emplace_back(vData[i]);
	}

	while (Variance < EPSILON)
	{
		Variance = 0;
		for (int i = 0; i < vData.size(); i++)
		{
			double MinDistance = DBL_MAX;
			int MinClusterIndex = -1;
			for (int j = 0; j < vK; j++)
			{
				double Temp = (vData[i] - ClusterCentroids[j]).norm();
				if (Temp < MinDistance)
				{
					MinDistance = Temp;
					MinClusterIndex = j;
				}
			}
			PointTag4Cluster[i] = MinClusterIndex;
		}

		auto calcentroid = [](const std::vector<Eigen::Vector3i>& vClusterPointsData) -> Eigen::Vector3i
		{
			Eigen::Vector3i Centroid(0, 0, 0);
			for (int i = 0; i < vClusterPointsData.size(); i++)
			{
				Centroid.x() += vClusterPointsData[i].x();
				Centroid.y() += vClusterPointsData[i].y();
				Centroid.z() += vClusterPointsData[i].z();
			}
			Centroid.x() = Centroid.x() / vClusterPointsData.size();
			Centroid.y() = Centroid.y() / vClusterPointsData.size();
			Centroid.z() = Centroid.z() / vClusterPointsData.size();

			return Centroid;
		};

		auto calvariance = [](const std::vector<Eigen::Vector3i>& vClusterPointsData, const Eigen::Vector3i& vCentroid) -> double
		{
			double Variance = 0;

			for (int j = 0; j < vClusterPointsData.size(); j++)
			{
				Variance += (vClusterPointsData[j] - vCentroid).squaredNorm();
			}
			Variance /= vClusterPointsData.size();
			return Variance;
		};

		for (int i = 0; i < vK; i++)
		{
			std::vector<Eigen::Vector3i> ClusterPointsData;
			for (int j = 0; j < vData.size(); j++)
			{
				if (PointTag4Cluster[j] == i)
					ClusterPointsData.emplace_back(vData[j]);
			}
			ClusterCentroids[i] = calcentroid(ClusterPointsData);
			Variance += calvariance(ClusterPointsData, ClusterCentroids[i]);
		}
		Variance /= vK;
	}
	return ClusterCentroids;

}



