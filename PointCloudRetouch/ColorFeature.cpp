#include "pch.h"
#include "ColorFeature.h"
#include "PointCloudRetouchManager.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CColorFeature, KEYWORD::COLOR_FEATURE)

//*****************************************************************
//FUNCTION: 
double CColorFeature::generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter)
{
	_ASSERTE(m_pConfig);
	m_ColorThreshold = *m_pConfig->getAttribute<float>("COLOR_THRESHOLD");
	m_NumMainColors = *m_pConfig->getAttribute<int>("NUM_MAIN_COLORS");
	
	if (vDeterminantPointSet.empty() || vValidationSet.empty())
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
std::vector<Eigen::Vector3i> CColorFeature::__kMeansCluster(const std::vector<Eigen::Vector3i>& vColorSet, std::size_t vK) const
{
	std::vector<Eigen::Vector3i*> PointTag4Cluster(vColorSet.size(), nullptr);
	std::vector<Eigen::Vector3i> ClusterCentroids;

	for (std::size_t i = 0; i < vK; i++)
		ClusterCentroids.push_back(vColorSet[i]);

	for (std::size_t i = 0; i < 30; i++)
	{
		for (size_t k = 0; k < vColorSet.size(); k++)
		{
			double MinDistance = DBL_MAX;
			Eigen::Vector3i* MinClusterPtr = nullptr;
			for (auto& Centroid : ClusterCentroids)
			{
				auto ColorDistance = (vColorSet[k] - Centroid).norm();
				if (ColorDistance < MinDistance && ColorDistance < m_ColorThreshold)
				{
					MinDistance = ColorDistance;
					MinClusterPtr = &Centroid;
				}
			}
			PointTag4Cluster[k] = MinClusterPtr;
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

		for (auto& Centroid : ClusterCentroids)
		{
			std::vector<Eigen::Vector3i> ClusterPointsData;
			for (int k = 0; k < vColorSet.size(); k++)
				if (PointTag4Cluster[k] == &Centroid)
					ClusterPointsData.push_back(vColorSet[k]);
			
			Centroid = calcentroid(ClusterPointsData);
		}
	}
	
	return ClusterCentroids;
}
