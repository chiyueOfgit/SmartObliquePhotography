#include "pch.h"

#include <common/MathInterface.h>
#include "ColorFeature.h"
#include "PlanarityFeature.h"
#include "PointCloudRetouchInterface.h"
#include "PointCloudRetouchConfig.h"

constexpr float EPSILON = 1e-4f;
constexpr float SPACE_SIZE = 100.0f;

//测试用例列表：
//ColorFeatureBaseTest随机生n种主颜色，要求K-Means聚类后生成的代表颜色均在对应颜色波动范围内（期望K为n）；
//  * Color_Feature_BaseTest_1:随机生成一种主颜色；
//  * Color_Feature_BaseTest_2:随机生成二种主颜色；
//  * Color_Feature_BaseTest_3:随机生成三种主颜色；
//  * Color_Feature_BaseTest_4:随机生成四种主颜色；
//ColorFeatureSpecialTest特定情况下的特殊结果正确
//  * Color_Feature_SpecialTest_1: ；

//PlaneFeatureBaseTest给定平面，随机生成带噪点的点云，要求以该点云拟合的平面与原平面差距在规定范围内；
//  * Plane_Feature_BaseTest_1: 随机模拟平面50%干扰点，期望平面夹角小于30；
//  * Plane_Feature_BaseTest_2: 随机模拟平面100%干扰点，期望平面夹角小于；
//PlaneFeatureSpecialTest特定情况下的特殊结果正确
//  * Plane_Feature_SpecialTest_1: ；


using namespace hiveObliquePhotography::PointCloudRetouch;
const std::string ConfigPath = "PointCloudRetouchConfig.xml";


PointCloud_t::PointType generateRandomPointByPlane(const Eigen::Vector4f& vPlane, bool vOnThePlane, float vNoise = 0.0f)
{
	Eigen::Vector4f Plane = vPlane;
	Plane /= Eigen::Vector3f(Plane.x(), Plane.y(), Plane.z()).eval().norm();

	auto RandomSet = hiveMath::hiveGenerateRandomRealSet(-SPACE_SIZE, SPACE_SIZE, 3);
	Eigen::Vector4f Point(RandomSet[0], RandomSet[1], RandomSet[2], 1.0f);

	if (!vOnThePlane)
	{
		float SignedDistance = Plane.dot(Point);
		if (abs(vNoise) > EPSILON)
			SignedDistance += hiveMath::hiveGenerateRandomReal(-vNoise, vNoise);
		//此后Point w分量失效
		Point -= SignedDistance * Plane;
	}

	PointCloud_t::PointType Output;
	for (size_t i = 0; i < 3; i++)
		Output.data[i] = Point[i];
	return Output;
}

Eigen::Vector4f generateRandomPlane()
{
	auto RandomSet = hiveMath::hiveGenerateRandomRealSet(-SPACE_SIZE, SPACE_SIZE, 4);

	return { RandomSet[0], RandomSet[1], RandomSet[2], RandomSet[3] };
}



void generateRandomColorSet(std::vector<Eigen::Vector3i>& vioColorCluster, Eigen::Vector3i& vMainColor, int vRange, int vNum)
{
	while (vNum--)
	{
		auto RandomSet = hiveMath::hiveGenerateRandomRealSet(0.0f, SPACE_SIZE, 1);
		int Offset = static_cast<int>(RandomSet[0]) % vRange;
		int Value = static_cast<int>(RandomSet[0]) % vRange;
		vMainColor[Offset] -= Value;
		vioColorCluster.push_back(vMainColor);
	}
}



TEST(Color_Feature_BaseTest_1, Test_1)
{
	hiveConfig::CHiveConfig* pConfig = new CPointCloudRetouchConfig;
	if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, pConfig) != hiveConfig::EParseResult::SUCCEED)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
		return;
	}
	std::vector<Eigen::Vector3i> Data;
	Eigen::Vector3i MainColor{ 255,3,3 };
	Eigen::Vector3i NoiseColor{ 100,100,100 };

	generateRandomColorSet(Data, MainColor, 3, 50);
	generateRandomColorSet(Data, NoiseColor, 30, 5);

	std::vector<Eigen::Vector3i> MainColorSet;
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<CColorFeature>(KEYWORD::COLOR_FEATURE, pConfig);
	//MainColorSet = pTileLoader->kMeansCluster(Data, 3);
	
	Eigen::Vector3i StandardColor{ 255,3,3 };
	int Sum = 0;
	for (auto& Color : MainColorSet)
		if ((Color - StandardColor).norm() > 5)
			Sum++;
	GTEST_ASSERT_EQ(Sum, 0);
}


TEST(Color_Feature_BaseTest_2, Test_2)
{
	hiveConfig::CHiveConfig* pConfig = new CPointCloudRetouchConfig;
	if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, pConfig) != hiveConfig::EParseResult::SUCCEED)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
		return;
	}
	std::vector<Eigen::Vector3i> Data;
	Eigen::Vector3i MainColor{ 255,3,3 };
	Eigen::Vector3i OtherColor{ 3,3,255 };
	Eigen::Vector3i NoiseColor{ 100,100,100 };
	
	generateRandomColorSet(Data, MainColor, 3, 50);
	generateRandomColorSet(Data, OtherColor, 3, 50);
	generateRandomColorSet(Data, NoiseColor, 30, 8);
	
	std::vector<Eigen::Vector3i> MainColorSet;
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<CColorFeature>(KEYWORD::COLOR_FEATURE, pConfig);
	//MainColorSet = pTileLoader->kMeansCluster(Data, 3);
	
	Eigen::Vector3i StandardColor{ 255,3,3 };
	Eigen::Vector3i OtherStandardColor{ 3,255,3 };
	int Sum = 0;
	for (auto& Color : MainColorSet)
		if ((Color - StandardColor).norm() < 3 || (Color - OtherStandardColor).norm() < 3)
			Sum++;
	GTEST_ASSERT_GE(Sum, 2);
}

TEST(Color_Feature_BaseTest_3, Test_3)
{
	
}

TEST(Color_Feature_BaseTest_4, Test_4)
{

}

TEST(Plane_Feature_BaseTest_1, Test_5)
{
	hiveConfig::CHiveConfig* pConfig = new CPointCloudRetouchConfig;
	if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, pConfig) != hiveConfig::EParseResult::SUCCEED)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
		return;
	}
	
	PointCloud_t::Ptr pCloud(new PointCloud_t);
	auto Plane = generateRandomPlane();

	for (size_t k = 0; k < 100; k++)
		pCloud->push_back(generateRandomPointByPlane(Plane, true));

	constexpr float OutlierFactor = 0.2f;
	for (size_t k = 0; k < OutlierFactor * 100; k++)
		pCloud->push_back(generateRandomPointByPlane(Plane, false));

	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<CPlanarityFeature>(KEYWORD::PLANARITY_FEATURE, pConfig);
	auto FittingPlane = pTileLoader->fitPlane(pCloud);

	Eigen::Vector3f PlaneNormal{ Plane[0],Plane[1],Plane[2]};
	PlaneNormal /= PlaneNormal.norm();
	Eigen::Vector3f FittingPlaneNormal{ FittingPlane[0],FittingPlane[1],FittingPlane[2] };
	
	GTEST_ASSERT_GE(abs(PlaneNormal.dot(FittingPlaneNormal)), 0.8);
}

TEST(Plane_Feature_BaseTest_2, Test_6)
{
	hiveConfig::CHiveConfig* pConfig = new CPointCloudRetouchConfig;
	if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, pConfig) != hiveConfig::EParseResult::SUCCEED)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
		return;
	}

	PointCloud_t::Ptr pCloud(new PointCloud_t);
	auto Plane = generateRandomPlane();

	for (size_t k = 0; k < 100; k++)
		pCloud->push_back(generateRandomPointByPlane(Plane, true));

	constexpr float OutlierFactor = 10.0f;
	for (size_t k = 0; k < OutlierFactor * 100; k++)
		pCloud->push_back(generateRandomPointByPlane(Plane, false));

	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<CPlanarityFeature>(KEYWORD::PLANARITY_FEATURE, pConfig);
	auto FittingPlane = pTileLoader->fitPlane(pCloud);

	Eigen::Vector3f PlaneNormal{ Plane[0],Plane[1],Plane[2] };
	PlaneNormal /= PlaneNormal.norm();
	Eigen::Vector3f FittingPlaneNormal{ FittingPlane[0],FittingPlane[1],FittingPlane[2] };
	
	GTEST_ASSERT_GE(abs(PlaneNormal.dot(FittingPlaneNormal)), 0.8);
}
