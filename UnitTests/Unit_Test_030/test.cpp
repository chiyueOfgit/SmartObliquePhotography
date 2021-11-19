#include "pch.h"

#include "PointCloudRetouchConfig.h"
#include "PointCloudRetouchManager.h"
#include "pcl/io/pcd_io.h"
#include <fstream>
#include "PointClusterExpanderMultithread.h"
#include <fstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include "AutoHoleRepairer.h"

#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

//≤‚ ‘”√¿˝¡–±Ì£∫
//  * 
//  * 
//	* 
//	* 

using namespace  hiveObliquePhotography::PointCloudRetouch;
using namespace  hiveObliquePhotography;

const std::string ConfigPath = TESTMODEL_DIR + std::string("Config/Test029_PointCloudRetouchConfig.xml");
const std::string PointCloudPath = std::string("D:/Models/Cloud/Tile_+004_+004_L23.ply");
const std::string ImagePath = TESTMODEL_DIR + std::string("Test029_Model/005005.png");

class TestHoleAutoRepair : public testing::Test
{
public:
	hiveConfig::CHiveConfig* pConfig = nullptr;
	CPointCloudRetouchManager* pManager = nullptr;
	std::vector<PointCloud_t::Ptr> TileSet;
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
		pcl::io::loadPLYFile(vModelPath, *pCloud);
		ASSERT_GT(pCloud->size(), 0);
		pManager = CPointCloudRetouchManager::getInstance();
		TileSet.push_back(pCloud);
		pManager->init(TileSet, pConfig);
	}

	void TearDown() override
	{
		delete pConfig;
	}
};

void _saveTexture(const std::string& vPath, const hiveObliquePhotography::CImage<float>& vTexture, bool vIsReverse)
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
			ResultImage[Offset] = vTexture.getColor(i, k);
		}

	stbi_write_png(vPath.c_str(), Width, Height, BytesPerPixel, ResultImage, 0);
	stbi_image_free(ResultImage);
}

void _loadTexture(const std::string& vPath, hiveObliquePhotography::CImage<float>& voTexture)
{
	int Width, Height, BytesPerPixel;
	unsigned char* Data = stbi_load(vPath.c_str(), &Width, &Height, &BytesPerPixel, 0);

	Eigen::Matrix<float, -1, -1> Image(Height, Width);
	for (int i = 0; i < Height; i++)
		for (int k = 0; k < Width; k++)
		{
			int Offset = (i * Width + k) * BytesPerPixel;
			Image(i, k) = Data[Offset];
		}

	voTexture.fillColor(Height, Width, Image.data());
}

void __serializeIndices(const std::vector<int>& vData, const std::string& vFileName)
{
	std::ofstream Out(vFileName);
	boost::archive::text_oarchive Oarchive(Out);
	Oarchive& BOOST_SERIALIZATION_NVP(vData);
	Out.close();
}

TEST_F(TestHoleAutoRepair, RepairImageByMipmap_Test_1)
{
	CImage<float> HoleImage;
	std::vector<Eigen::Vector2i> HoleSet;
	_loadTexture("Hole.png", HoleImage);
	CAutoHoleRepairer AutoHoleRepairer;
	AutoHoleRepairer.repairImageByMipmap(HoleImage, HoleSet);
	_saveTexture("WithoutHole.png", HoleImage,false);
}

TEST_F(TestHoleAutoRepair, RepairImageByMipmap_Test_2)
{
	/*{
		Eigen::Matrix<float, -1, -1> BlackSet(4, 4);
		for (int i = 0; i < 4; i++)
			for (int k = 0; k < 4; k++)
				BlackSet(k, i) = { 0 };
		BlackSet(0, 0) = 100; BlackSet(1, 0) = 100; BlackSet(1, 1) = 100; BlackSet(2, 0) = 100; BlackSet(3, 0) = 100; BlackSet(3, 1) = 100;
		BlackSet(2, 3) = 250; BlackSet(3, 2) = 250; BlackSet(3, 3) = 250;
		CImage<float> MaskImage;
		MaskImage.fillColor(4, 4, BlackSet.data());
		_saveTexture("Input.png", MaskImage, false);
	}*/
	/*{
		Eigen::Matrix<float, -1, -1> BlackSet(4, 4);
		BlackSet(0, 0) = 100; BlackSet(1, 0) = 100; BlackSet(1, 1) = 100; BlackSet(0, 1) = 100;
		BlackSet(2, 0) = 100; BlackSet(3, 0) = 100; BlackSet(3, 1) = 100; BlackSet(2, 1) = 100;
		BlackSet(2, 3) = 250; BlackSet(3, 2) = 250; BlackSet(3, 3) = 250; BlackSet(2, 2) = 250;
		BlackSet(0, 2) = 150; BlackSet(0, 3) = 150; BlackSet(1, 2) = 150; BlackSet(1, 3) = 150;
		CImage<float> MaskImage;
		MaskImage.fillColor(4, 4, BlackSet.data());
		_saveTexture("GroundTruth.png", MaskImage, false);
	}*/
	
}

