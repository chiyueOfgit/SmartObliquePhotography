#include "pch.h"
#include "PointCloudAutoRetouchScene.h"
#include "RegionGrowingByColorAlg.h"
#include <set>


using namespace hiveObliquePhotography::AutoRetouch;

SRegionGrowingSetting RegionGrowingSetting;



void CRegionGrowingByColorAlg::runV(const std::vector<std::uint64_t>& vSeedSet, EPointLabel vSeedLabel) //SeedLabel是什么？
{

	std::vector<std::uint64_t> Seeds(vSeedSet.begin(), vSeedSet.end());

	auto pPointCloudScene = CPointCloudAutoRetouchScene::getInstance();
	auto pScene = pPointCloudScene->getPointCloudScene();
	auto pKdTree = pPointCloudScene->getGlobalKdTree();

	std::vector<int> PointLabels;
	constexpr int InitLabelValue = -1;
	PointLabels.resize(pScene->size(), InitLabelValue);

	//calculate average color and median color
	bool bColorFlag = true;   //下一步写到配置文件里
	if (bColorFlag)
	{
		m_AverageColor.first.resize(3);
		for (auto Index : Seeds)
		{
			PointLabels.at(Index) = 0;
			m_pLocalLabelSet->changePointLabel(Index, vSeedLabel);

			if (SRegionGrowingSetting::EColorMode::Mean == SRegionGrowingSetting::EColorMode::Mean)// 下一步改写到配置文件获取
			{
				m_AverageColor.first[0] += (*pScene)[Index].r;
				m_AverageColor.first[1] += (*pScene)[Index].g;
				m_AverageColor.first[2] += (*pScene)[Index].b;

				m_AverageColor.second++;
			}
			else
			{
				m_MortonCode.insert(m_MortonCode.end(), Morton_3d(pScene->points[Index].r, pScene->points[Index].g, pScene->points[Index].b));
			}
		}
		if (SRegionGrowingSetting::EColorMode::Mean == SRegionGrowingSetting::EColorMode::Median)// 下一步改写到配置文件获取
		{
			//initialization median
			sort(m_MortonCode.begin(), m_MortonCode.end());
			if (m_MortonCode.size())
				inverse_Morton_3d(m_MedianColor[0], m_MedianColor[1], m_MedianColor[2], m_MortonCode[m_MortonCode.size() / 2]);
		}
	}

	while (!Seeds.empty())
	{
		int KNN = 0;

		const int CurrentSeed = Seeds.back();
		Seeds.pop_back();

		std::vector<int> NeighborIndices;
		std::vector<float> NeighborDistances;

		pKdTree->radiusSearch(pScene->points[CurrentSeed], RegionGrowingSetting.SearchSize, NeighborIndices, NeighborDistances);
		while (KNN < NeighborIndices.size())
		{
			int NeighborIndex = NeighborIndices[KNN];
			//跳过已聚类的点
			if (PointLabels[NeighborIndex] != -1)
			{
				KNN++;
				continue;
			}

			//这里认为半径内其中一个测试点代表所有测试点是否直接通过
			if (__validatePoint(NeighborIndex, pScene))
			{
				KNN = 0;
				while (KNN < NeighborIndices.size())
				{
					if (PointLabels[NeighborIndex] == -1)
					{
						int NeighborIndex = NeighborIndices[KNN];
						Seeds.push_back(NeighborIndex);
						PointLabels[NeighborIndex] = 0;
						m_pLocalLabelSet->changePointLabel(NeighborIndex, vSeedLabel);
					}
					KNN++;
				}

				break;
			}

			KNN++;
		}
	}

}

bool CRegionGrowingByColorAlg::__validatePoint(int vTestPoint, const pcl::PointCloud<pcl::PointSurfel>::Ptr& vCloud) const
{
	if (RegionGrowingSetting.bColorFlag)
	{
		switch (RegionGrowingSetting.ColorMode)
		{
		case SRegionGrowingSetting::EColorMode::Mean:
			if (!__colorTestByAverage(vTestPoint, vCloud))
				return false;
			break;
		case SRegionGrowingSetting::EColorMode::Median:
			if (!__colorTestByMedian(vTestPoint, vCloud))
				return false;
			break;
		}
	}

	if (RegionGrowingSetting.bGroundFlag)
	{
		if (!__groundTest(vTestPoint, vCloud))
			return false;
	}

	if (RegionGrowingSetting.bNormalFlag)
	{
		if (!__normalTest(vTestPoint, vCloud))
			return false;
	}

	return true;
}

float CRegionGrowingByColorAlg::__calculateColorimetricalDifference(std::vector<unsigned int>& vFirstColor, std::vector<unsigned int>& vSecondColor) const
{
	float Difference = 0.0f;
	Difference += (vFirstColor[0] - vSecondColor[0]) * (vFirstColor[0] - vSecondColor[0]);
	Difference += (vFirstColor[1] - vSecondColor[1]) * (vFirstColor[1] - vSecondColor[1]);
	Difference += (vFirstColor[2] - vSecondColor[2]) * (vFirstColor[2] - vSecondColor[2]);
	return sqrtf(Difference);
}

bool CRegionGrowingByColorAlg::__colorTestByAverage(int vTestPoint, const pcl::PointCloud<pcl::PointSurfel>::Ptr& vCloud) const
{
	//应该需要动态更新阈值
	float ColorThreshold = RegionGrowingSetting.ColorThreshold;

	//用平均颜色代表整体颜色
	std::vector<unsigned int> OverallColor(3);
	OverallColor[0] = m_AverageColor.first[0] / (float)m_AverageColor.second;
	OverallColor[1] = m_AverageColor.first[1] / (float)m_AverageColor.second;
	OverallColor[2] = m_AverageColor.first[2] / (float)m_AverageColor.second;

	//用测试点的邻居平均颜色代表该测试点的颜色
	std::vector<int> NeighborIndices;
	std::vector<float> NeighborDistances;
	auto pKdTree = CPointCloudAutoRetouchScene::getInstance()->getGlobalKdTree();
	pKdTree->radiusSearch((*vCloud)[vTestPoint], RegionGrowingSetting.SearchSize, NeighborIndices, NeighborDistances);

	std::vector<unsigned int> NeighborColor(3);
	for (auto Index : NeighborIndices)
	{
		NeighborColor[0] += (*vCloud)[Index].r;
		NeighborColor[1] += (*vCloud)[Index].g;
		NeighborColor[2] += (*vCloud)[Index].b;
	}

	NeighborColor[0] = NeighborColor[0] / (float)NeighborIndices.size();
	NeighborColor[1] = NeighborColor[1] / (float)NeighborIndices.size();
	NeighborColor[2] = NeighborColor[2] / (float)NeighborIndices.size();

	if (m_ColorDifferenceCalculator.calcColorDifferences(OverallColor, NeighborColor) > ColorThreshold)
	{
		//		ColorThreshold--;
		return false;
	}
	else
	{
		//通过的更新平均颜色
		m_AverageColor.first[0] += NeighborColor[0];
		m_AverageColor.first[1] += NeighborColor[1];
		m_AverageColor.first[2] += NeighborColor[2];

		m_AverageColor.second++;
	}

	return true;
}

bool CRegionGrowingByColorAlg::__colorTestByMedian(int vTestPoint, const pcl::PointCloud<pcl::PointSurfel>::Ptr& vCloud) const
{
	//应该需要动态更新阈值
	float ColorThreshold = RegionGrowingSetting.ColorThreshold;

	//用测试点的邻居平均颜色代表该测试点的颜色
	std::vector<int> NeighborIndices;
	std::vector<float> NeighborDistances;
	auto pKdTree = CPointCloudAutoRetouchScene::getInstance()->getGlobalKdTree();
	pKdTree->radiusSearch((*vCloud)[vTestPoint], RegionGrowingSetting.SearchSize, NeighborIndices, NeighborDistances);

	std::vector<unsigned int> NeighborColor(3);
	for (auto Index : NeighborIndices)
	{
		NeighborColor[0] += (*vCloud)[Index].r;
		NeighborColor[1] += (*vCloud)[Index].g;
		NeighborColor[2] += (*vCloud)[Index].b;
	}

	NeighborColor[0] = NeighborColor[0] / (float)NeighborIndices.size();
	NeighborColor[1] = NeighborColor[1] / (float)NeighborIndices.size();
	NeighborColor[2] = NeighborColor[2] / (float)NeighborIndices.size();

	if (m_ColorDifferenceCalculator.calcColorDifferences(m_MedianColor, NeighborColor) > ColorThreshold)
	{
		//		ColorThreshold--;
		return false;
	}
	else
	{
		m_MortonCode.insert(m_MortonCode.end(), Morton_3d(vCloud->points[vTestPoint].r, vCloud->points[vTestPoint].g, vCloud->points[vTestPoint].b));
		sort(m_MortonCode.begin(), m_MortonCode.end());
		inverse_Morton_3d(m_MedianColor[0], m_MedianColor[1], m_MedianColor[2], m_MortonCode[m_MortonCode.size() / 2]);
	}

	return true;
}

bool CRegionGrowingByColorAlg::__groundTest(int vTestPoint, const pcl::PointCloud<pcl::PointSurfel>::Ptr& vCloud) const
{
	if ((*vCloud)[vTestPoint].z < RegionGrowingSetting.GroundHeight)
		return false;
	else
		return true;
}

bool CRegionGrowingByColorAlg::__normalTest(int vTestPoint, const pcl::PointCloud<pcl::PointSurfel>::Ptr& vCloud) const
{
	std::vector<int> NeighborIndices;
	std::vector<float> NeighborDistances;

	float OuterRadius = 2.0f;
	float InnerRadius = 1.0f;
	auto pKdTree = CPointCloudAutoRetouchScene::getInstance()->getGlobalKdTree();
	pKdTree->radiusSearch((*vCloud)[vTestPoint], OuterRadius, NeighborIndices, NeighborDistances);
	std::set<int> OutIndices(NeighborIndices.begin(), NeighborIndices.end());

	NeighborIndices.clear();
	NeighborDistances.clear();
	pKdTree->radiusSearch((*vCloud)[vTestPoint], InnerRadius, NeighborIndices, NeighborDistances);
	std::set<int> InIndices(NeighborIndices.begin(), NeighborIndices.end());

	std::vector<int> Difference(OutIndices.size() - InIndices.size(), -1);
	std::set_difference(OutIndices.begin(), OutIndices.end(), InIndices.begin(), InIndices.end(), Difference.begin());

	std::vector<float> NeighborDot;
	NeighborDot.reserve(OutIndices.size() - InIndices.size());
	float SumDot = 0.0f;
	auto Iter = Difference.begin();
	while (Iter != Difference.end() && *Iter != -1)
	{
		std::size_t Index = *Iter;
		float Dot = (*vCloud)[Index].normal_x * (*vCloud)[vTestPoint].normal_x + (*vCloud)[Index].normal_y * (*vCloud)[vTestPoint].normal_y + (*vCloud)[Index].normal_z * (*vCloud)[vTestPoint].normal_z;
		NeighborDot.push_back(Dot);
		SumDot += Dot;

		Iter++;
	}

	float NormalThreshold = 0.4f * (OutIndices.size() - InIndices.size());
	if (SumDot <= NormalThreshold)
		return false;
	else
		return true;
}
