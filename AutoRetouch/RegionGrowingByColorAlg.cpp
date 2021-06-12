#include "pch.h"
#include <set>
#include "PointCloudAutoRetouchScene.h"
#include "RegionGrowingByColorAlg.h"
#include "AutoRetouchConfig.h"

using namespace hiveObliquePhotography::AutoRetouch;

_REGISTER_EXCLUSIVE_PRODUCT(CRegionGrowingByColorAlg, CLASSIFIER_REGION_GROW_COLOR)

//*****************************************************************
//FUNCTION: 
void CRegionGrowingByColorAlg::__initValidation(const pcl::Indices& vSeeds, PointCloud_t::ConstPtr vCloud)
{
	m_ColorTestMode = static_cast<EColorMode>(*CAutoRetouchConfig::getInstance()->getAttribute<int>("COLOR_TEST_MODE"));
	m_EnableColorTest = *CAutoRetouchConfig::getInstance()->getAttribute<bool>("ENABLE_COLOR_TEST");
	m_EnableGroundTest = *CAutoRetouchConfig::getInstance()->getAttribute<bool>("ENABLE_GROUND_TEST");
	m_EnableNormalTest = *CAutoRetouchConfig::getInstance()->getAttribute<bool>("ENABLE_NORMAL_TEST");

	if (m_EnableColorTest)
	{
		m_AverageColor[0] = m_AverageColor[1] = m_AverageColor[2] = 0;
		m_AverageColor[3] = static_cast<int>(vSeeds.size());
		m_SeedsSize = vSeeds.size();
		switch (m_ColorTestMode)
		{
		case EColorMode::MEAN:
			{
				const auto ColorMap = vCloud->getMatrixXfMap(1,
					sizeof(PointCloud_t::PointType) / sizeof(float),
					offsetof(PointCloud_t::PointType, data_c) / sizeof(float));
				m_SeedsAverageColor = *reinterpret_cast<const std::uint32_t*>(ColorMap.rowwise().mean().eval().data());
			}
			break;
		case EColorMode::MEDIAN:
			{
				for (auto Index : vSeeds)
					m_MortonCodes.push_back(__morton4(vCloud->at(Index).rgba));
				std::sort(m_MortonCodes.begin(), m_MortonCodes.end());
				if (!m_MortonCodes.empty())
					m_MedianColor = __inverseMorton4(m_MortonCodes[m_MortonCodes.size() / 2]);
			}
			break;
		}
	}
}

//*****************************************************************
//FUNCTION: 
bool CRegionGrowingByColorAlg::__validatePointV(pcl::index_t vTestIndex, PointCloud_t::ConstPtr vCloud) const
{
	//if (m_EnableColorTest)
	//{
	//	switch (static_cast<EColorMode>(CAutoRetouchConfig::getInstance()->getAttribute<int>("COLOR_TEST_MODE").value()))
	//	{
	//	case EColorMode::MEAN:
	//		if (!__colorTestByAverage(vTestIndex, vCloud))
	//			return false;
	//		break;
	//	case EColorMode::MEDIAN:
	//		if (!__colorTestByMedian(vTestIndex, vCloud))
	//			return false;
	//		break;
	//	}
	//}
	//
	//if (m_EnableGroundTest)
	//	if (!__groundTest(vTestIndex, vCloud))
	//		return false;
	//
	//if (m_EnableNormalTest)
	//	if (!__normalTest(vTestIndex, vCloud))
	//		return false;

	return true;
}

//*****************************************************************
//FUNCTION: 
float CRegionGrowingByColorAlg::__calculateColorimetricalDifference(std::uint32_t vFirstColor[3], std::uint32_t vSecondColor[3]) const
{
	float Difference = 0.0f;
	Difference += (vFirstColor[0] - vSecondColor[0]) * (vFirstColor[0] - vSecondColor[0]);
	Difference += (vFirstColor[1] - vSecondColor[1]) * (vFirstColor[1] - vSecondColor[1]);
	Difference += (vFirstColor[2] - vSecondColor[2]) * (vFirstColor[2] - vSecondColor[2]);
	return sqrtf(Difference);
}

//*****************************************************************
//FUNCTION: 
bool CRegionGrowingByColorAlg::__colorTestByAverage(int vTestIndex, PointCloud_t::ConstPtr vCloud) const
{
	//应该需要动态更新阈值
	float ColorThreshold = CAutoRetouchConfig::getInstance()->getAttribute<float>("COLOR_TEST_THRESHOLD").value();

	//用平均颜色代表整体颜色
	//用测试点的邻居平均颜色代表该测试点的颜色
	std::vector<int> NeighborIndices;
	std::vector<float> NeighborDistances;
	const auto pTree = CPointCloudAutoRetouchScene::getInstance()->getGlobalKdTree();
	pTree->radiusSearch((*vCloud)[vTestIndex], *CAutoRetouchConfig::getInstance()->getAttribute<float>("SEARCH_RADIUS"), NeighborIndices, NeighborDistances);

	std::vector<unsigned int> NeighborColor(3, 0);
	for (auto Index : NeighborIndices)
	{
		NeighborColor[0] += (*vCloud)[Index].r;
		NeighborColor[1] += (*vCloud)[Index].g;
		NeighborColor[2] += (*vCloud)[Index].b;
	}

	NeighborColor[0] /= NeighborIndices.size();
	NeighborColor[1] /= NeighborIndices.size();
	NeighborColor[2] /= NeighborIndices.size();

	auto [r, g, b, a] = __extractColor(m_SeedsAverageColor);
	std::vector<std::uint32_t> AverageColor{ r, g, b };
	for (auto i : AverageColor)
		i = (i * m_SeedsSize + m_AverageColor[i]) / (m_SeedsSize + m_AverageColor[3]);
	
	if (ColorDifferences::calcColorDifferences(AverageColor, NeighborColor) > ColorThreshold)
	{
		//ColorThreshold--;
		return false;
	}
	else
	{
		//通过的更新平均颜色
		m_AverageColor[0] += NeighborColor[0];
		m_AverageColor[1] += NeighborColor[1];
		m_AverageColor[2] += NeighborColor[2];

		++m_AverageColor[3];
	}

	return true;
}

//*****************************************************************
//FUNCTION: 
bool CRegionGrowingByColorAlg::__colorTestByMedian(int vTestIndex, PointCloud_t::ConstPtr vCloud) const
{
	//应该需要动态更新阈值
	float ColorThreshold = CAutoRetouchConfig::getInstance()->getAttribute<float>("COLOR_TEST_THRESHOLD").value();

	//用测试点的邻居平均颜色代表该测试点的颜色
	std::vector<int> NeighborIndices;
	std::vector<float> NeighborDistances;
	auto pTree = CPointCloudAutoRetouchScene::getInstance()->getGlobalKdTree();
	pTree->radiusSearch((*vCloud)[vTestIndex], CAutoRetouchConfig::getInstance()->getAttribute<float>("SEARCH_RADIUS").value(), NeighborIndices, NeighborDistances);

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

	auto [r, g, b, a] = __extractColor(m_MedianColor);
	std::vector<std::uint32_t> MedianColor{r, g, b};
	if (ColorDifferences::calcColorDifferences(MedianColor, NeighborColor) > ColorThreshold)
	{
		//ColorThreshold--;
		return false;
	}
	else
	{
		m_MortonCodes.push_back(__morton4(vCloud->at(vTestIndex).rgba));
		std::sort(m_MortonCodes.begin(), m_MortonCodes.end());
		m_MedianColor = __inverseMorton4(m_MortonCodes[m_MortonCodes.size() / 2]);
	}

	return true;
}

//*****************************************************************
//FUNCTION: 
bool CRegionGrowingByColorAlg::__groundTest(int vTestIndex, PointCloud_t::ConstPtr vCloud) const
{
	if ((*vCloud)[vTestIndex].z < CAutoRetouchConfig::getInstance()->getAttribute<float>("GROUND_TEST_THRESHOLD").value())
		return false;
	else
		return true;
}

//*****************************************************************
//FUNCTION: 
bool CRegionGrowingByColorAlg::__normalTest(int vTestIndex, PointCloud_t::ConstPtr vCloud) const
{
	std::vector<int> NeighborIndices;
	std::vector<float> NeighborDistances;

	float OuterRadius = 2.0f;
	float InnerRadius = 1.0f;
	auto pTree = CPointCloudAutoRetouchScene::getInstance()->getGlobalKdTree();
	pTree->radiusSearch((*vCloud)[vTestIndex], OuterRadius, NeighborIndices, NeighborDistances);
	std::set<int> OutIndices(NeighborIndices.begin(), NeighborIndices.end());

	NeighborIndices.clear();
	NeighborDistances.clear();
	pTree->radiusSearch((*vCloud)[vTestIndex], InnerRadius, NeighborIndices, NeighborDistances);
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
		float Dot = (*vCloud)[Index].normal_x * (*vCloud)[vTestIndex].normal_x + (*vCloud)[Index].normal_y * (*vCloud)[vTestIndex].normal_y + (*vCloud)[Index].normal_z * (*vCloud)[vTestIndex].normal_z;
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

//*****************************************************************
//FUNCTION: 
std::uint32_t CRegionGrowingByColorAlg::__morton4(std::uint32_t vData)
{
	std::uint32_t Sum = 0;
	for (int i = 0; i < 4; i++)
	{
		std::uint32_t Component = vData >> i * 8;

		Component &= 0x000000FF;
		Component = (Component | (Component << 12)) & 0x000F000F;
		Component = (Component | (Component << 6)) & 0x03030303;
		Component = (Component | (Component << 3)) & 0x11111111;

		Sum |= Component << i;
	}
	return Sum;
}

//*****************************************************************
//FUNCTION: 
std::uint32_t CRegionGrowingByColorAlg::__inverseMorton4(std::uint32_t vMorton)
{
	std::uint32_t Sum = 0;
	for (int i = 0; i < 4; i++)
	{
		std::uint32_t Component = vMorton >> i;

		Component &= 0x11111111;
		Component = ((Component >> 3) | Component) & 0x03030303;
		Component = ((Component >> 6) | Component) & 0x000F000F;
		Component = ((Component >> 12) | Component) & 0x000000FF;

		Sum |= Component << i * 8;
	}
	return Sum;
}

//*****************************************************************
//FUNCTION: 
std::tuple<std::uint8_t, std::uint8_t, std::uint8_t, std::uint8_t> CRegionGrowingByColorAlg::__extractColor(std::uint32_t vColor)
{
	PointCloud_t::PointType Point;
	std::memcpy(&Point.rgba, &vColor, sizeof(std::uint32_t));
	
	return { Point.r, Point.g, Point.b, Point.a };
}
