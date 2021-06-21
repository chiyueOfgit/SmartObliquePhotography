#include "pch.h"
#include "ColorFeature.h"
#include "PointCloudRetouchManager.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CColorFeature, KEYWORD::COLOR_FEATURE)

#define eps 1e-12

//*****************************************************************
//FUNCTION: 
double CColorFeature::generateFeatureV(const std::vector<pcl::index_t>& vDeterminantPointSet, const std::vector<pcl::index_t>& vValidationSet, pcl::index_t vClusterCenter)
{
	//todo: ÅäÖÃÎÄ¼þµÄK
	__computeMainColors(vDeterminantPointSet, m_MainColors, 3);
	std::vector<Eigen::Vector3i> ValidationColors;
	__computeMainColors(vValidationSet, ValidationColors, 3);


}

//*****************************************************************
//FUNCTION: 
double CColorFeature::evaluateFeatureMatchFactorV(pcl::index_t vInputPoint)
{
	
}

//*****************************************************************
//FUNCTION: 
void CColorFeature::__computeMainColors(const std::vector<pcl::index_t>& vPointIndices, std::vector<Eigen::Vector3i>& vMainColors, std::size_t vK)
{
	std::vector<Eigen::Vector3i> PointCloudColors;

	for (auto Index : vPointIndices)
	{
		PointCloudColors.push_back(CPointCloudRetouchManager::getInstance()->getRetouchScene().getColorAt(Index));
	}

	vMainColors = __kMeansCluster(PointCloudColors, vK);
}

//*****************************************************************
//FUNCTION: 
std::vector<Eigen::Vector3i> CColorFeature::__kMeansCluster(const std::vector<Eigen::Vector3i>& vData, std::size_t vK) const
{
	std::vector<int> number(vData.size());
	std::vector<Eigen::Vector3i> base;
	double variance = 0;

	for (int i = 0; i < vK; i++)
	{
		base.emplace_back(vData[i]);
	}

	while (variance < eps)
	{
		variance = 0;
		for (int i = 0; i < vData.size(); i++)
		{
			int temp = 0;
			double dis = 0;
			for (int j = 0; j < vK; j++)
			{
				double temp = (vData[i] - base[j]).norm();
				if (dis > temp)
				{
					dis = temp;
					number[i] = j;
				}
			}

		}

		auto calcentroid = [](const std::vector<Eigen::Vector3i>& temp) -> Eigen::Vector3i
		{
			Eigen::Vector3i ans(0, 0, 0);
			for (int i = 0; i < temp.size(); i++)
			{
				ans.x() += temp[i].x();
				ans.y() += temp[i].y();
				ans.z() += temp[i].z();
			}
			ans.x() = ans.x() / temp.size();
			ans.y() = ans.y() / temp.size();
			ans.z() = ans.z() / temp.size();

			return ans;
		};

		auto calvariance = [](std::vector<Eigen::Vector3i>& temp, Eigen::Vector3i& base) -> double
		{
			double variance = 0;

			for (int j = 0; j < temp.size(); j++)
			{
				variance += (temp[j] - base).squaredNorm();
			}
			variance /= temp.size();
			return variance;
		};

		for (int i = 0; i < vK; i++)
		{
			std::vector<Eigen::Vector3i> temp;
			for (int j = 0; j < vData.size(); j++)
			{
				if (number[j] == i)
					temp.emplace_back(vData[j]);
			}
			base[i] = calcentroid(temp);
			variance += calvariance(temp, base[i]);
		}
		variance /= vK;
	}
	return base;

}



