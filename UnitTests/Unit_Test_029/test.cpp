#include "pch.h"

#include "GroundObjectExtractor.h"
#include "PointCloudRetouchConfig.h"
#include "PointCloudRetouchInterface.h"
#include "PointCloudRetouchManager.h"
#include "pcl/io/pcd_io.h"
#include <fstream>

#include "stb_image.h"
#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

//测试用例列表：
//  * OverallTest_generateElevationMap_1: 测试根据点云生成高程图的正确性
//  * PartTest_calc
//	* 
//	* 

using namespace  hiveObliquePhotography::PointCloudRetouch;
using namespace  hiveObliquePhotography;

const std::string ConfigPath = TESTMODEL_DIR + std::string("Config/Test029_PointCloudRetouchConfig.xml");

class TestOutlierDetector : public testing::Test
{
public:
	hiveConfig::CHiveConfig* pConfig = nullptr;
	CPointCloudRetouchManager* pManager = nullptr;
protected:

	void SetUp() override
	{
		
		pConfig = new CPointCloudRetouchConfig;
		if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, pConfig) != hiveConfig::EParseResult::SUCCEED)
		{
			_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
			return;
		}
;	}

	void initTest(const std::string& vModelPath)
	{
		PointCloud_t::Ptr pCloud(new PointCloud_t);
		pcl::io::loadPCDFile(vModelPath, *pCloud);
		ASSERT_GT(pCloud->size(), 0);
		pManager = CPointCloudRetouchManager::getInstance();
		std::vector<PointCloud_t::Ptr> vTileSet;
		vTileSet.push_back(pCloud);
		pManager->init(vTileSet, pConfig);
	}

	void TearDown() override
	{
		delete pConfig;
	}
};
void _saveTexture(const std::string& vPath, const hiveObliquePhotography::CImage<std::array<int, 1>>& vTexture, bool vIsReverse)
{
	const auto Width = vTexture.getWidth();
	const auto Height = vTexture.getHeight();
	const auto BytesPerPixel = 1;
	auto ResultImage = new unsigned char[Width * Height * BytesPerPixel];
	for (auto i = 0; i < Height; i++)
		for (auto k = 0; k < Width; k++)
		{
			auto I = i;
			if (vIsReverse)
				I = Height - 1 - I;
			auto Offset = (I * Width + k) * BytesPerPixel;
			ResultImage[Offset] = vTexture.getColor(i, k)[0];
			//ResultImage[Offset + 1] = vTexture.getColor(i, k)[1];
			//ResultImage[Offset + 2] = vTexture.getColor(i, k)[2];
		}

	stbi_write_png(vPath.c_str(), Width, Height, BytesPerPixel, ResultImage, 0);
	stbi_image_free(ResultImage);
}

//TEST_F(TestOutlierDetector, OverallTest_generateElevationMap_1)
//{
//	initTest(TESTMODEL_DIR + std::string("Test029_Model/test1.pcd"));
//	auto pExtractor = hiveDesignPattern::hiveCreateProduct<CGroundObjectExtractor>(KEYWORD::GROUND_OBJECT_EXTRACTOR);
//	EXPECT_NE(pExtractor, nullptr);
//	if (!pExtractor)
//		std::cerr << "create baker error." << std::endl;
//	Eigen::Vector2i Resolution{ 10, 10 };
//	CImage<std::array<int, 1>> ResultImage;
//	ResultImage = pExtractor->generateElevationMap(Resolution);
//
//	auto PNGFilePath = TESTMODEL_DIR + std::string("Test029_Model/100RG.png");
//	int Channels = 3;
//	unsigned char* GtData = stbi_load(PNGFilePath.c_str(), &Resolution.x(), &Resolution.y(), &Channels, 0);
//	for (int i = 0; i < Resolution.x() * Resolution.y(); i++)
//	{
//		std::array<int, 3> GtColor{ GtData[i * 3], GtData[i * 3 + 1], GtData[i * 3 + 2] };
//		auto Color = ResultImage.getColor(i / Resolution.x(), i % Resolution.x());
//		EXPECT_EQ(Color, GtColor);
//	}
//}

TEST_F(TestOutlierDetector, GenerateMap)
{
	initTest(TESTMODEL_DIR + std::string("Test029_Model/005004.pcd"));
	auto pExtractor = hiveDesignPattern::hiveCreateProduct<CGroundObjectExtractor>(KEYWORD::GROUND_OBJECT_EXTRACTOR);
	EXPECT_NE(pExtractor, nullptr);
	if (!pExtractor)
		std::cerr << "create baker error." << std::endl;
	Eigen::Vector2i Resolution{ 1024, 1024 };
	CImage<std::array<int, 1>> ResultImage;
	ResultImage = pExtractor->generateElevationMap(Resolution);
	
	_saveTexture("005004.png", ResultImage, false);
}