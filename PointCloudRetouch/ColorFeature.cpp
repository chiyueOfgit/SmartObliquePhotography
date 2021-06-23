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

		if ((Color - MainColor).norm() <= m_ColorThreshold)
			return 1.0;
		else if((Color - MainColor).norm() < m_ColorThreshold * 2)
		{
			auto a = (Color - MainColor).norm();
			return 1.0 / ((Color - MainColor).norm() - m_ColorThreshold);
		}
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
	std::vector<std::pair<Eigen::Vector3i*, Eigen::Vector3i>> TagAndColorSet(vColorSet.size(), { nullptr, Eigen::Vector3i() });
	for (size_t i = 0; i < TagAndColorSet.size(); i++)
	{
		auto& [Tag, Color] = TagAndColorSet[i];
		Color = vColorSet[i];
	}
	
	std::vector<Eigen::Vector3i> ClusterCentroids(vK, Eigen::Vector3i());
	for (auto& Centroid : ClusterCentroids)
		Centroid = TagAndColorSet[hiveMath::hiveGenerateRandomInteger(std::size_t(0), TagAndColorSet.size() - 1)].second;

	for (std::size_t i = 0; i < 30; i++)
	{		
		for (auto& [Tag, Color] : TagAndColorSet)
		{
			std::pair<float, Eigen::Vector3i*> MinPair(FLT_MAX, nullptr);
			for (auto& Centroid : ClusterCentroids)
			{
				float ColorDistance = (Color - Centroid).norm();

				if (ColorDistance < m_ColorThreshold)
					MinPair = std::min(MinPair, std::make_pair(ColorDistance, &Centroid));
			}
			Tag = MinPair.second;
		}

		auto calcentroid = [](const std::vector<Eigen::Vector3i>& vClusterColorSet) -> Eigen::Vector3i
		{
			Eigen::Vector3i Centroid(0, 0, 0);
			if (vClusterColorSet.empty())
				return Centroid;
			
			for (auto& Color : vClusterColorSet)
				Centroid += Color;
			Centroid /= vClusterColorSet.size();
			return Centroid;
		};

		for (auto& Centroid : ClusterCentroids)
		{
			std::vector<Eigen::Vector3i> ClusterPointsData;
			for (auto& [Tag, Color] : TagAndColorSet)
				if (Tag == &Centroid)
					ClusterPointsData.push_back(Color);
			Centroid = calcentroid(ClusterPointsData);
		}
	}
	
	return ClusterCentroids;
}
