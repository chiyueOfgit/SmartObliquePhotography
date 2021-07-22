#include "pch.h"

#include <common/MathInterface.h>
#include "ColorFeature.h"
#include "PlanarityFeature.h"

#include "PointCloudRetouchInterface.h"
#include "PointCloudRetouchConfig.h"
#include "NormalComplexity.h"

#include <fstream>
#include <boost/archive/text_oarchive.hpp> 
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>

#include "PointCloudRetouchManager.h"
#include "PointClusterExpander.h"
#include "PointCluster.h";

constexpr float EPSILON = 1e-4f;
constexpr float SPACE_SIZE = 100.0f;

//测试用例列表：
//ColorFeatureBaseTest随机生n种主颜色，要求K-Means聚类后生成的代表颜色均在对应颜色波动范围内（期望K为n）；
//  * Color_Feature_BaseTest_1:随机生成一种主颜色；
//  * Color_Feature_BaseTest_2:随机生成二种主颜色；
//  * Color_Feature_BaseTest_3:随机生成三种主颜色；
//  * Color_Feature_BaseTest_4:随机生成四种主颜色；
//  * Color_Feature_BaseTest_5:主颜色越多生长范围越大；
//ColorFeatureSpecialTest特定情况下的特殊结果正确
//  * Color_Feature_SpecialTest_1: ；

//PlaneFeatureBaseTest给定平面，随机生成带噪点的点云，要求以该点云拟合的平面与原平面差距在规定范围内；
//  * Plane_Feature_BaseTest_1: 随机模拟平面20%干扰点，期望平面夹角小于10°；
//  * Plane_Feature_BaseTest_2: 随机模拟平面50%干扰点，期望平面夹角小于30°；
//PlaneFeatureSpecialTest特定情况下的特殊结果正确
//  * Plane_Feature_SpecialTest_1: ；

//NormalFeatureBaseTest给定限定生成的场景，要求计算的点的法线差异在规定范围内
//  * Normal_Feature_BaseTest_1: 在点支持半径内随机给定法线一致点，在支持半径外生成法线扰乱点，要求该点法线差异为0；
//  * Normal_Feature_BaseTest_2: 在点周围生成法线在一定范围扰动的点，使用pcl法线估计器进行大小支持半径法线估计，得到GT，要求点法线差异与GT差别小于0.1；
//  * Normal_Feature_BaseTest_3: 选定树木区域与地面区域，要求算得树木区域法线差异大于地面区域；
//NormalComplexitySpecialTest特定情况下的特殊结果正确
//  * Plane_Feature_SpecialTest_1: ；


using namespace hiveObliquePhotography::PointCloudRetouch;
const std::string ConfigPath = TESTMODEL_DIR + std::string("Config/Test015_PointCloudRetouchConfig_Simple.xml");
const std::string PickedIndicesPath = TESTMODEL_DIR + std::string("Test015_Model/PickedIndices.txt");
const std::string CameraPath = TESTMODEL_DIR + std::string("Test015_Model/Camera.txt");
const std::string PointCloudPath = TESTMODEL_DIR + std::string("General/slice 16.pcd");
const std::string CompleteConfigPath = TESTMODEL_DIR + std::string("Config/Test015_PointCloudRetouchConfig_Complete.xml");


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

PointCloud_t::PointType generateNoisePoint()
{
	auto RandomSet = hiveMath::hiveGenerateRandomRealSet(-SPACE_SIZE, SPACE_SIZE, 3);
	PointCloud_t::PointType Output;
	for (size_t i = 0; i < 3; i++)
		Output.data[i] = RandomSet[i];
	return Output;
}

Eigen::Vector4f generateRandomPlane()
{
	auto RandomSet = hiveMath::hiveGenerateRandomRealSet(-SPACE_SIZE, SPACE_SIZE, 4);

	return { RandomSet[0], RandomSet[1], RandomSet[2], RandomSet[3] };
}

void generateRandomColorSet(std::vector<Eigen::Vector3i>& vioColorCluster, const Eigen::Vector3i& vMainColor, int vRange, int vNum)
{
	while (vNum--)
	{
		auto TempColor = vMainColor;
		auto RandomSet = hiveMath::hiveGenerateRandomRealSet(0.0f, SPACE_SIZE, 1);
		int Offset = static_cast<int>(RandomSet[0]) % vRange;
		int Value = static_cast<int>(RandomSet[0]) % vRange;
		TempColor[Offset] -= Value;

		vioColorCluster.push_back(TempColor);
	}
}

void generateNoiseColorSet(std::vector<Eigen::Vector3i>& vioColorCluster, int vNum)
{
	while (vNum--)
	{
		auto RandomSet = hiveMath::hiveGenerateRandomRealSet(0.0f, 255.0f, 3);
		Eigen::Vector3i TempColor{ static_cast<int>(RandomSet[0]),static_cast<int>(RandomSet[1]),static_cast<int>(RandomSet[2]) };
		vioColorCluster.push_back(TempColor);
	}
}

Eigen::Vector3f generatePosition(Eigen::Vector3f& vCenterPosition, float vFrom, float vTo, bool vOnThePlane)
{
	float SmallOffset = sqrt(vFrom * vFrom / 3);
	float LargeOffset = sqrt(vTo * vTo / 3);
	auto RandomSet = hiveMath::hiveGenerateRandomRealSet(SmallOffset, LargeOffset, 3);
	if(vOnThePlane)
		return Eigen::Vector3f{ vCenterPosition[0] + RandomSet[0],vCenterPosition[1] + RandomSet[1],vCenterPosition[2]};
	else
	    return Eigen::Vector3f{ vCenterPosition[0] + RandomSet[0],vCenterPosition[1] + RandomSet[1],vCenterPosition[2] + RandomSet[2] / 2 };
}

Eigen::Vector3f generateNormal(Eigen::Vector3f& vStandardNormal, float vDisturb)
{
	auto RandomSet = hiveMath::hiveGenerateRandomRealSet(-vDisturb, vDisturb, 3);
	Eigen::Vector3f Normal{ vStandardNormal[0] + RandomSet[0],vStandardNormal[1] + RandomSet[1],vStandardNormal[2] + RandomSet[2] };
	return Normal / Normal.norm();
}

void generateInOutRadiusPoint(Eigen::Vector3f& vCenterPosition, float vFrom, float vTo, bool vOnThePlane, float vDisturb, PointCloud_t& vioPointSet, int vNum)
{
	Eigen::Vector3f StandardNormal{ 0.0f,0.0f,1.0f };
	while (vNum--)
	{
		Eigen::Vector3f Position = generatePosition(vCenterPosition, vFrom, vTo, vOnThePlane);
		Eigen::Vector3f Normal = generateNormal(StandardNormal, vDisturb);
		pcl::PointSurfel Temp;
		Temp.x = Position[0];
		Temp.y = Position[1];
		Temp.z = Position[2];
		Temp.normal_x = Normal[0];
		Temp.normal_y = Normal[1];
		Temp.normal_z = Normal[2];
		vioPointSet.push_back(Temp);
	}
}

void loadIndices(const std::string& vPath, pcl::Indices& voIndices)
{
	std::ifstream File(vPath);
	boost::archive::text_iarchive ia(File);
	ia >> BOOST_SERIALIZATION_NVP(voIndices);
	File.close();
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
	Eigen::Vector3i MainColor{ 125,125,125 };

	generateRandomColorSet(Data, MainColor, 3, 50);
	generateNoiseColorSet(Data, 5);

	std::vector<Eigen::Vector3i> MainColorSet;
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<CColorFeature>(KEYWORD::COLOR_FEATURE);
	pTileLoader->initV(pConfig);
	
	MainColorSet = pTileLoader->adjustKMeansCluster(Data, 6);
	
	int Sum = 0;
	for (auto& Color : MainColorSet)
		if ((Color - MainColor).norm() > 10)
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
	Eigen::Vector3i MainColor{ 100,150,100 };
	Eigen::Vector3i OtherColor{ 200,150,200 };
	
	generateRandomColorSet(Data, MainColor, 3, 50);
	generateRandomColorSet(Data, OtherColor, 3, 50);
	generateNoiseColorSet(Data, 8);
	
	std::vector<Eigen::Vector3i> MainColorSet;
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<CColorFeature>(KEYWORD::COLOR_FEATURE);
	pTileLoader->initV(pConfig);
	MainColorSet = pTileLoader->adjustKMeansCluster(Data, 6);
	
	int Sum = 0;
	for (auto& Color : MainColorSet)
		if ((Color - MainColor).norm() < 10 || (Color - OtherColor).norm() < 10)
			Sum++;
	GTEST_ASSERT_GE(Sum, 2);
}

TEST(Color_Feature_BaseTest_3, Test_3)
{
	hiveConfig::CHiveConfig* pConfig = new CPointCloudRetouchConfig;
	if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, pConfig) != hiveConfig::EParseResult::SUCCEED)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
		return;
	}
	std::vector<Eigen::Vector3i> Data;
	Eigen::Vector3i MainColor{ 70,140,70 };
	Eigen::Vector3i OtherColor{ 140,70,140 };
	Eigen::Vector3i AnotherColor{ 70,140,210 };
	
	generateRandomColorSet(Data, MainColor, 3, 50);
	generateRandomColorSet(Data, OtherColor, 3, 50);
	generateRandomColorSet(Data, AnotherColor, 3, 50);
	generateNoiseColorSet(Data, 20);

	std::vector<Eigen::Vector3i> MainColorSet;
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<CColorFeature>(KEYWORD::COLOR_FEATURE);
	pTileLoader->initV(pConfig);
	MainColorSet = pTileLoader->adjustKMeansCluster(Data, 6);
	
	int Sum = 0;
	for (auto& Color : MainColorSet)
		if ((Color - MainColor).norm() < 8 || (Color - OtherColor).norm() < 8 || (Color - AnotherColor).norm() < 8)
			Sum++;
	GTEST_ASSERT_GE(Sum, 3);
}

TEST(Color_Feature_BaseTest_4, Test_4)
{
	hiveConfig::CHiveConfig* pConfig = new CPointCloudRetouchConfig;
	if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, pConfig) != hiveConfig::EParseResult::SUCCEED)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
		return;
	}
	std::vector<Eigen::Vector3i> Data;
	Eigen::Vector3i MainColor{ 70,140,70 };
	Eigen::Vector3i OtherColor{ 140,70,140 };
	Eigen::Vector3i AnotherColor{ 70,140,210 };
	Eigen::Vector3i ForthColor{ 125,125,125 };

	generateRandomColorSet(Data, MainColor, 3, 50);
	generateRandomColorSet(Data, OtherColor, 3, 50);
	generateRandomColorSet(Data, AnotherColor, 3, 50);
	generateRandomColorSet(Data, ForthColor, 3, 50);
	generateNoiseColorSet(Data, 30);

	std::vector<Eigen::Vector3i> MainColorSet;
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<CColorFeature>(KEYWORD::COLOR_FEATURE);
	pTileLoader->initV(pConfig);
	MainColorSet = pTileLoader->adjustKMeansCluster(Data, 6);
	
	int Sum = 0;
	for (auto& Color : MainColorSet)
		if ((Color - MainColor).norm() < 8 || (Color - OtherColor).norm() < 8 || (Color - AnotherColor).norm() < 8 || (Color - ForthColor).norm() < 8)
			Sum++;
	GTEST_ASSERT_GE(Sum, 4);
}

TEST(Color_Feature_BaseTest_5, Test_5)
{
	bool result = hiveUtility::hiveFileSystem::hiveIsFileExisted(PickedIndicesPath);

	pcl::Indices PickedIndices;
	std::ifstream File(PickedIndicesPath);
	boost::archive::text_iarchive ia(File);
	ia >> BOOST_SERIALIZATION_NVP(PickedIndices);
	File.close();

	PointCloud_t::Ptr pCloud(new PointCloud_t);
	pcl::io::loadPCDFile(PointCloudPath, *pCloud);

	hiveConfig::CHiveConfig* pCompleteConfig = new CPointCloudRetouchConfig;
	if (hiveConfig::hiveParseConfig(CompleteConfigPath, hiveConfig::EConfigType::XML, pCompleteConfig) != hiveConfig::EParseResult::SUCCEED)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", CompleteConfigPath));
		return;
	}

	pcl::visualization::Camera Camera;
	auto pVisualizer = new pcl::visualization::PCLVisualizer("Viewer", true);
	pVisualizer->loadCameraParameters(CameraPath);
	pVisualizer->getCameraParameters(Camera);
	Eigen::Matrix4d ViewMatrix, ProjectionMatrix;
	Camera.computeViewMatrix(ViewMatrix);
	Camera.computeProjectionMatrix(ProjectionMatrix);

	double Hardness = 0.4;
	double RadiusOnWindow = 55.321;
	Eigen::Vector2d CircleCenterOnWindow = { 534, 228 };

	auto HardnessFunc = [=](const Eigen::Vector2d& vPos) -> double
	{
		Eigen::Vector2d PosOnWindow((vPos.x() + 1) * Camera.window_size[0] / 2, (vPos.y() + 1) * Camera.window_size[1] / 2);

		double X = (PosOnWindow - CircleCenterOnWindow).norm() / RadiusOnWindow;
		if (X <= 1.0)
		{
			X -= Hardness;
			if (X < 0)
				return 1.0;
			X /= (1 - Hardness);
			X *= X;

			return X * (X - 2) + 1;
		}
		else
			return 0;
	};

	auto pManager = CPointCloudRetouchManager::getInstance();
	pManager->init(pCloud, pCompleteConfig);

	hiveConfig::CHiveConfig* pConfig = new CPointCloudRetouchConfig;
	if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, pConfig) != hiveConfig::EParseResult::SUCCEED)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
		return;
	}

	int SmallHardnessColorNum = 0, LargeHardnessColorNum = 0;

	auto pFunc = [&](int& vNum)
	{
		auto pCluster = pManager->generateInitialCluster(PickedIndices, ProjectionMatrix * ViewMatrix, HardnessFunc, EPointLabel::UNWANTED);
		auto& Indices = pCluster->getCoreRegion();
		std::vector<Eigen::Vector3i> ColorSet;
		for (auto Index : Indices)
			ColorSet.push_back({ pCloud->points[Index].r, pCloud->points[Index].g, pCloud->points[Index].b });

		auto* pColorFeature = hiveDesignPattern::hiveGetOrCreateProduct<CColorFeature>(KEYWORD::COLOR_FEATURE);
		pColorFeature->initV(pConfig);
		const auto& MainColorSet = pColorFeature->adjustKMeansCluster(ColorSet, 5);
		vNum = MainColorSet.size();
	};
	
	pFunc(SmallHardnessColorNum);
	pManager->executeMarker(PickedIndices, ProjectionMatrix * ViewMatrix, HardnessFunc, EPointLabel::UNWANTED);
	auto SmallHardnessExpandPointsNumber = pManager->getLitterMarker().getExpander()->getExpandPoints().size();

	Hardness = 0.7;
	pFunc(LargeHardnessColorNum);
	pManager->executeMarker(PickedIndices, ProjectionMatrix * ViewMatrix, HardnessFunc, EPointLabel::UNWANTED);
	auto LargeHardnessExpandPointsNumber = pManager->getLitterMarker().getExpander()->getExpandPoints().size();

	//生长范围只和主颜色数量相关，与硬度非直接关系	fixme: 实现更大区域(可能更平缓)找出更多主颜色
	if (SmallHardnessColorNum < LargeHardnessColorNum)
		EXPECT_GE(LargeHardnessExpandPointsNumber, SmallHardnessExpandPointsNumber);
	else
		EXPECT_LE(LargeHardnessExpandPointsNumber, SmallHardnessExpandPointsNumber);
}

TEST(Plane_Feature_BaseTest_1, Test_6)
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
		//pCloud->push_back(generateNoisePoint());

	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<CPlanarityFeature>(KEYWORD::PLANARITY_FEATURE);
	pTileLoader->initV(pConfig);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pPositionCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*pCloud, *pPositionCloud);
	auto FittingPlane = pTileLoader->fitPlane(pPositionCloud, 1.0, {0, 0, 1});

	Eigen::Vector3f PlaneNormal{ Plane[0],Plane[1],Plane[2]};
	PlaneNormal /= PlaneNormal.norm();
	Eigen::Vector3f FittingPlaneNormal{ FittingPlane[0],FittingPlane[1],FittingPlane[2] };
	
	GTEST_ASSERT_GE(abs(PlaneNormal.dot(FittingPlaneNormal)), 0.8);
}

TEST(Plane_Feature_BaseTest_2, Test_7)
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

	constexpr float OutlierFactor = 0.4f;
	for (size_t k = 0; k < OutlierFactor * 100; k++)
		pCloud->push_back(generateRandomPointByPlane(Plane, false));
		//pCloud->push_back(generateNoisePoint());

	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<CPlanarityFeature>(KEYWORD::PLANARITY_FEATURE);
	pTileLoader->initV(pConfig);
	pcl::PointCloud<pcl::PointXYZ>::Ptr pPositionCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*pCloud, *pPositionCloud);
	auto FittingPlane = pTileLoader->fitPlane(pPositionCloud, 1.0, { 0, 0, 1 });

	Eigen::Vector3f PlaneNormal{ Plane[0],Plane[1],Plane[2] };
	PlaneNormal /= PlaneNormal.norm();
	Eigen::Vector3f FittingPlaneNormal{ FittingPlane[0],FittingPlane[1],FittingPlane[2] };
	
	GTEST_ASSERT_GE(abs(PlaneNormal.dot(FittingPlaneNormal)), 0.8);
}

TEST(Normal_Feature_BaseTest_1, Test_8)
{
	hiveConfig::CHiveConfig* pConfig = new CPointCloudRetouchConfig;
	if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, pConfig) != hiveConfig::EParseResult::SUCCEED)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
		return;
	}

	std::string Path = TESTMODEL_DIR + std::string("Config/Test015_PointCloudRetouchConfig_Complete.xml");
	hiveConfig::CHiveConfig* pTestConfig = new CPointCloudRetouchConfig;
	if (hiveConfig::hiveParseConfig(Path, hiveConfig::EConfigType::XML, pTestConfig) != hiveConfig::EParseResult::SUCCEED)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", Path));
		return;
	}

	PointCloud_t::Ptr pCloud(new PointCloud_t);
	Eigen::Vector3f GTPosition{ 0.0f,0.0f,0.0f };
	Eigen::Vector3f GTNormal{ 0.0f,0.0f,1.0f };
	GTNormal /= GTNormal.norm();
	pcl::PointSurfel Temp;
	Temp.x = GTPosition[0];
	Temp.y = GTPosition[1];
	Temp.z = GTPosition[2];
	Temp.normal_x = GTNormal[0];
	Temp.normal_y = GTNormal[1];
	Temp.normal_z = GTNormal[2];
	pCloud->push_back(Temp);
	auto Radius = *pConfig->getAttribute<double>("LARGE_SCALE_RADIUS");
	generateInOutRadiusPoint(GTPosition, 0, Radius, true,0.0f, *pCloud, 20);
	generateInOutRadiusPoint(GTPosition, Radius + 2, Radius + 4,true, 0.4f, *pCloud, 5);

	CPointCloudRetouchManager* pManager = nullptr;
	pManager = CPointCloudRetouchManager::getInstance();
	pManager->init(pCloud, pTestConfig);
	
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<CNormalComplexity>(KEYWORD::NORMAL_COMPLEXITY);
	pTileLoader->initV(pConfig);
	auto Res = pTileLoader->calcSinglePointNormalComplexity(0);

	GTEST_ASSERT_EQ(Res, 0.0);
}

TEST(Normal_Feature_BaseTest_2, Test_9)
{
	hiveConfig::CHiveConfig* pConfig = new CPointCloudRetouchConfig;
	if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, pConfig) != hiveConfig::EParseResult::SUCCEED)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
		return;
	}

	std::string Path = TESTMODEL_DIR + std::string("Config/Test015_PointCloudRetouchConfig_Complete.xml");
	hiveConfig::CHiveConfig* pTestConfig = new CPointCloudRetouchConfig;
	if (hiveConfig::hiveParseConfig(Path, hiveConfig::EConfigType::XML, pTestConfig) != hiveConfig::EParseResult::SUCCEED)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", Path));
		return;
	}

	PointCloud_t::Ptr pCloud(new PointCloud_t);
	Eigen::Vector3f GTPosition{ 0.0f,0.0f,0.0f };
	Eigen::Vector3f GTNormal{ 0.0f,0.0f,1.0f };
	GTNormal /= GTNormal.norm();
	pcl::PointSurfel ThisPoint;
	ThisPoint.x = GTPosition[0];
	ThisPoint.y = GTPosition[1];
	ThisPoint.z = GTPosition[2];
	ThisPoint.normal_x = GTNormal[0];
	ThisPoint.normal_y = GTNormal[1];
	ThisPoint.normal_z = GTNormal[2];
	pCloud->push_back(ThisPoint);
	auto Radius = *pConfig->getAttribute<double>("LARGE_SCALE_RADIUS");
	generateInOutRadiusPoint(GTPosition, 0, Radius, false,0.4f, *pCloud, 20);

	CPointCloudRetouchManager* pManager = nullptr;
	pManager = CPointCloudRetouchManager::getInstance();
	pManager->init(pCloud, pTestConfig);
	
	pcl::PointCloud<pcl::Normal>::Ptr OtherNormals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointSurfel, pcl::Normal> OtherNormalEstimation;
	OtherNormalEstimation.setInputCloud(pCloud);
	OtherNormalEstimation.setRadiusSearch(Radius);
	pcl::search::KdTree<pcl::PointSurfel>::Ptr OtherKdtree(new pcl::search::KdTree<pcl::PointSurfel>);
	OtherNormalEstimation.setSearchMethod(OtherKdtree);
	OtherNormalEstimation.compute(*OtherNormals);

	Eigen::Vector3f OutNormal{ OtherNormals->points[0].normal_x,OtherNormals->points[0].normal_y ,OtherNormals->points[0].normal_z };
	OutNormal /= OutNormal.norm();
	
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<CNormalComplexity>(KEYWORD::NORMAL_COMPLEXITY);
	pTileLoader->initV(pConfig);
	auto Res = pTileLoader->calcSinglePointNormalComplexity(0);

	double Diff = GTNormal.dot(OutNormal);

	auto GT = sqrt(1 - Diff * Diff);
	GTEST_ASSERT_LT(abs(Res - GT), 0.3);
	
}

TEST(Normal_Feature_BaseTest_3, Test_10)
{
	hiveConfig::CHiveConfig* pConfig = new CPointCloudRetouchConfig;
	if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, pConfig) != hiveConfig::EParseResult::SUCCEED)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
		return;
	}

	std::string Path = TESTMODEL_DIR + std::string("Config/Test015_PointCloudRetouchConfig_Complete.xml");
	hiveConfig::CHiveConfig* pTestConfig = new CPointCloudRetouchConfig;
	if (hiveConfig::hiveParseConfig(Path, hiveConfig::EConfigType::XML, pTestConfig) != hiveConfig::EParseResult::SUCCEED)
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", Path));
		return;
	}

	pcl::Indices Tree;
	loadIndices(TESTMODEL_DIR + std::string("Test017_Model/CompleteTree/CompleteTreeGT.txt"), Tree);

	pcl::Indices Ground;
	loadIndices(TESTMODEL_DIR + std::string("Test017_Model/KeepGround/CompleteGroundGT.txt"), Ground);

	std::string ModelPath(TESTMODEL_DIR + std::string("General/slice 16.pcd"));
	PointCloud_t::Ptr pCloud(new PointCloud_t);
	pcl::io::loadPCDFile(ModelPath, *pCloud);

	CPointCloudRetouchManager* pManager = nullptr;
	pManager = CPointCloudRetouchManager::getInstance();
	pManager->init(pCloud, pTestConfig);
	
	auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<CNormalComplexity>(KEYWORD::NORMAL_COMPLEXITY);
	pTileLoader->initV(pConfig);
	double ResTree = 0.0;
	for(auto Index: Tree)
	{
		ResTree += pTileLoader->calcSinglePointNormalComplexity(Index);
	}
	ResTree /= Tree.size();

	double ResGround = 0.0;
	for (auto Index : Ground)
	{
		ResGround += pTileLoader->calcSinglePointNormalComplexity(Index);
	}
	ResGround /= Ground.size();

	GTEST_ASSERT_LT(ResGround, ResTree);
}