#include "pch.h"
#include "ColorFeature.h"
#include "PointCloudRetouchInterface.h"
#include "PointCloudRetouchConfig.h"

//测试用例列表：
//  * Color_Feature_Test_1:随机生成无主颜色；
//  * Color_Feature_Test_1:随机生成一种主颜色；
//  * Color_Feature_Test_2:随机生成二种主颜色；
//  * Color_Feature_Test_3:随机生成四种主颜色；

//  * Plane_Feature_Test_1: 随机模拟平面30%干扰点；
//  * Plane_Feature_Test_2: ；
using namespace hiveObliquePhotography::PointCloudRetouch;
const std::string ConfigPath = "PointCloudRetouchConfig.xml";

//TEST(Color_Feature_Test_1, Test_1)
//{
//	hiveConfig::CHiveConfig* vFeatureConfig;
//	std::vector<Eigen::Vector3i> Data{};
//	std::vector<Eigen::Vector3i> MainColorSet;
//	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<CColorFeature>(KEYWORD::COLOR_FEATURE, vFeatureConfig);
//	MainColorSet = pTileLoader->kMeansCluster(Data, 3);
//}


TEST(Color_Feature_Test_2, Test_2)
{
	hiveConfig::CHiveConfig* pConfig = new CPointCloudRetouchConfig;
	if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, pConfig) != hiveConfig::EParseResult::SUCCEED)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
		return;
	}
	srand((int)time(0));
	int MainNum = 100,OtherNum = 30;
	std::vector<Eigen::Vector3i> Data;
	while(MainNum--)
	{
		Eigen::Vector3i MainColor{ 255,3,3 };
		int Offset = rand() % 3;
		int Value = rand() % 3;
		MainColor[Offset] -= Value;
		Data.push_back(MainColor);
	}
	while (OtherNum--)
	{
		Eigen::Vector3i OtherColor{ 3,3,255 };
		int Offset = rand() % 3;
		int Value = rand() % 3;
		OtherColor[Offset] -= Value;
		Data.push_back(OtherColor);
	}
	std::vector<Eigen::Vector3i> MainColorSet;
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<CColorFeature>(KEYWORD::COLOR_FEATURE, pConfig);
	MainColorSet = pTileLoader->kMeansCluster(Data, 3);
	Eigen::Vector3i StandardColor{ 255,3,3 };
	int Sum = 0;
	for (auto& Color : MainColorSet)
		if ((Color - StandardColor).norm() > 5)
			Sum++;
	GTEST_ASSERT_EQ(Sum, 0);
}

TEST(Color_Feature_Test_3, Test_3)
{
	hiveConfig::CHiveConfig* pConfig = new CPointCloudRetouchConfig;
	if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, pConfig) != hiveConfig::EParseResult::SUCCEED)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
		return;
	}
	
	srand((int)time(0));
	int MainNum = 100, OtherMainNum = 100, OtherNum = 30;
	std::vector<Eigen::Vector3i> Data;
	while (MainNum--)
	{
		Eigen::Vector3i MainColor{ 255,3,3 };
		int Offset = rand() % 3;
		int Value = rand() % 3;
		MainColor[Offset] -= Value;
		Data.push_back(MainColor);
	}
	while (OtherMainNum--)
	{
		Eigen::Vector3i MainColor{ 3,255,3 };
		int Offset = rand() % 3;
		int Value = rand() % 3;
		MainColor[Offset] -= Value;
		Data.push_back(MainColor);
	}
	while (OtherNum--)
	{
		Eigen::Vector3i OtherColor{ 3,3,255 };
		int Offset = rand() % 3;
		int Value = rand() % 3;
		OtherColor[Offset] -= Value;
		Data.push_back(OtherColor);
	}
	std::vector<Eigen::Vector3i> MainColorSet;
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<CColorFeature>(KEYWORD::COLOR_FEATURE, pConfig);
	MainColorSet = pTileLoader->kMeansCluster(Data, 3);
	Eigen::Vector3i StandardColor{ 255,3,3 };
	Eigen::Vector3i OtherStandardColor{ 3,255,3 };
	int Sum = 0;
	for (auto& Color : MainColorSet)
		if ((Color - StandardColor).norm() < 3 || (Color - OtherStandardColor).norm() < 3)
			Sum++;
	GTEST_ASSERT_GE(Sum, 2);
}

//TEST(Plane_Feature_Test_1, Test_4)
//{
//	
//}