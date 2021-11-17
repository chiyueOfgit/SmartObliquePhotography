#include "pch.h"

#include "GroundObjectExtractor.h"
#include "PointCloudRetouchConfig.h"
#include "PointCloudRetouchInterface.h"
#include "PointCloudRetouchManager.h"
#include "pcl/io/pcd_io.h"
#include <fstream>
#include "PointCluster.h"
#include "PointClusterSet.h"
#include "PointClusterExpanderMultithread.h"
#include <fstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <vtkTIFFWriter.h>
#include <vtkTIFFReader.h>
#include <vtkSmartPointer.h>
#include <vtkImageViewer2.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkImageData.h> 
#include <vtkImageShiftScale.h>

#include "vtkAutoInit.h"
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

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
const std::string PointCloudPath = std::string("D:/Models/Cloud/Tile_+004_+004_L23.ply");
const std::string ImagePath = TESTMODEL_DIR + std::string("Test029_Model/005005.png");

class TestObjectExtractor : public testing::Test
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

void _loadTexture(const std::string& vPath, hiveObliquePhotography::CImage<std::array<int, 1>>& voTexture)
{
	int Width, Height, BytesPerPixel;
	unsigned char* Data = stbi_load(vPath.c_str(), &Width, &Height, &BytesPerPixel, 0);

	Eigen::Matrix<std::array<int, 1>, -1, -1> Image(Height, Width);
	for (int i = 0; i < Height; i++)
		for (int k = 0; k < Width; k++)
		{
			int Offset = (i * Width + k) * BytesPerPixel;
			Image(i, k)[0] = Data[Offset];
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

//TEST_F(TestObjectExtractor, OverallTest_generateElevationMap_1)
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

//TEST_F(TestObjectExtractor, GenerateMap)
//{
//	initTest(PointCloudPath);
//	auto pExtractor = hiveDesignPattern::hiveCreateProduct<CGroundObjectExtractor>(KEYWORD::GROUND_OBJECT_EXTRACTOR);
//	EXPECT_NE(pExtractor, nullptr);
//	if (!pExtractor)
//		std::cerr << "create Extractor error." << std::endl;
//	Eigen::Vector2i Resolution{ 1024, 1024 };
//	CImage<std::array<int, 1>> ResultImage;
//	ResultImage = pExtractor->generateElevationMap(Resolution);
//	auto GrownImage = pExtractor->generateGrownImage(ResultImage);
//	auto EdgeImage = pExtractor->generateEdgeImage(GrownImage);
//	auto EdgeSet = pExtractor->divide2EdgeSet(EdgeImage);
//	
//	_saveTexture("ResultImage.png", ResultImage, false);
//	_saveTexture("GrownImage.png", GrownImage, false);
//	_saveTexture("EdgeImage.png", EdgeImage, false);
//}

//TEST_F(TestObjectExtractor, OutPutIndices)
//{
//	initTest(PointCloudPath);
//
//	Eigen::Vector2i Resolution{ 1024,1024 };
//	std::vector<pcl::index_t> OutPutIndices;
//	std::vector<std::vector<pcl::index_t>> EdgeIndices;
//	std::vector<pcl::index_t> SumSet;
//	
//	auto pExtractor = hiveDesignPattern::hiveCreateProduct<CGroundObjectExtractor>(KEYWORD::GROUND_OBJECT_EXTRACTOR);
//	EXPECT_NE(pExtractor, nullptr);
//	if (!pExtractor)
//		std::cerr << "create Extractor error." << std::endl;
//
//    pExtractor->execute<CGroundObjectExtractor>(OutPutIndices, EdgeIndices, Resolution);
//	for(auto& Edges: EdgeIndices)
//	{
//		SumSet.insert(SumSet.end(), Edges.begin(), Edges.end());
//	}
//
//	__serializeIndices(OutPutIndices, "005005SeedObject.txt");
//	__serializeIndices(SumSet, "005005EdgeIndices.txt");
//}

//TEST_F(TestObjectExtractor, Growing)
//{
//	CImage<std::array<int, 1>> Texture;
//	initTest(PointCloudPath);
//	_loadTexture(ImagePath, Texture);
//
//	std::vector<pcl::index_t> FeatureGenerationSet, ValidationSet;
//
//	auto pExtractor = hiveDesignPattern::hiveCreateProduct<CGroundObjectExtractor>(KEYWORD::GROUND_OBJECT_EXTRACTOR);
//	EXPECT_NE(pExtractor, nullptr);
//	if (!pExtractor)
//		std::cerr << "create Extractor error." << std::endl;
//	pExtractor->map2Cloud(Texture, FeatureGenerationSet);
//
//	CPointCluster* pInitialCluster = new CPointCluster;
//	const hiveConfig::CHiveConfig* pClusterConfig = pManager->getLitterMarker().getClusterConfig();;
//
//	std::vector<pcl::index_t>::iterator Iter = FeatureGenerationSet.begin();
//	ValidationSet.push_back(*Iter);
//	FeatureGenerationSet.erase(Iter);
//
//	CPointClusterExpanderMultithread* pPointClusterExpanderMultithread = new CPointClusterExpanderMultithread;
//	pInitialCluster->init(pClusterConfig, 0, EPointLabel::UNWANTED, FeatureGenerationSet, ValidationSet, pManager->addAndGetTimestamp());
//	pPointClusterExpanderMultithread->execute<CPointClusterExpanderMultithread>(pInitialCluster);
//
//	/*std::vector<std::size_t> PointLabel;
//	PointCloudRetouch::hiveDumpTileLabel(WhichTile, PointLabel);
//	m_pVisualizer->refresh(WhichTile, PointLabel);*/
//}

//TEST_F(TestObjectExtractor, OutputGround)
//{
//	CImage<std::array<int, 1>> InputImage;
//	initTest(PointCloudPath);
//	_loadTexture(ImagePath, InputImage);
//
//	Eigen::Vector2i Resolution{ 1024, 1024 };
//	std::vector<pcl::index_t> OutputIndices;
//
//	auto pExtractor = hiveDesignPattern::hiveCreateProduct<CGroundObjectExtractor>(KEYWORD::GROUND_OBJECT_EXTRACTOR);
//	EXPECT_NE(pExtractor, nullptr);
//	if (!pExtractor)
//		std::cerr << "Create Extractor error." << std::endl;
//	pExtractor->map2Cloud(InputImage, OutputIndices, false);
//
//	__serializeIndices(OutputIndices, "005005SeedGround.txt");
//}

TEST()
{
	const std::string TiffPath = "xiang.tiff";
	const std::string PngPath = "ElevatioMap.png";
	vtkSmartPointer<vtkTIFFReader> reader = vtkSmartPointer<vtkTIFFReader>::New();
	reader->SetFileName(TiffPath.c_str());
	reader->Update();
	/*vtkSmartPointer<vtkImageViewer2> imageViewer = vtkSmartPointer<vtkImageViewer2>::New();
	imageViewer->SetInputConnection(reader->GetOutputPort());
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	imageViewer->SetupInteractor(renderWindowInteractor);
	imageViewer->Render();
	imageViewer->GetRenderer()->ResetCamera();
	imageViewer->Render();
	renderWindowInteractor->Start();*/

	//获取图像维数
	int dims[3];
	reader->GetOutput()->GetDimensions(dims);

	cout << "dims[0-3]: " << dims[0] << "   " << dims[1] << "   " << dims[2] << endl << endl;

	int extent[6];
	double spacing[3];
	double origin[3];
	reader->GetOutput()->GetExtent(extent);
	reader->GetOutput()->GetSpacing(spacing);
	reader->GetOutput()->GetOrigin(origin);

	// 计算中心位置。  
	double center[3];
	center[0] = origin[0] + spacing[0] * 0.5 * (extent[0] + extent[1]);
	center[1] = origin[1] + spacing[1] * 0.5 * (extent[2] + extent[3]);
	center[2] = origin[2] + spacing[2] * 0.5 * (extent[4] + extent[5]);

	cout << "坐标原点";
	cout << "origin[0]: " << origin[0] << endl;
	cout << "origin[1]: " << origin[1] << endl;
	cout << "origin[2]: " << origin[2] << endl << endl;

	cout << "像素间隔";
	cout << "spacing[0]: " << spacing[0] << endl;
	cout << "spacing[1]: " << spacing[1] << endl;
	cout << "spacing[2]: " << spacing[2] << endl << endl;

	cout << "第一维数据范围extent[0-1]: " << extent[0] << "  " << extent[1] << endl;
	cout << "第二维数据范围extent[0-1]: " << extent[2] << "  " << extent[3] << endl;
	cout << "第三维数据范围extent[0-1]: " << extent[4] << "  " << extent[5] << endl;

	cout << "图片中间";
	cout << "center[0]: " << center[0] << endl;
	cout << "center[1]: " << center[0] << endl;
	cout << "center[2]: " << center[0] << endl << endl;


	//打印原像素
	int p = 0;
	cout << "[ red, green, blue ]" << endl;
	for (int k = 0; k < dims[2]; k++)
	{
		for (int j = 0; j < dims[1]; j++)
		{
			for (int i = 0; i < dims[0]; i++)
			{
				if (i > 1500 && i < 1511 && j > 1500 && j < 1511)    //10 * 10个像素,图片中间
				{
					unsigned char* pixel = (unsigned char*)(reader->GetOutput()->GetScalarPointer(i, j, k));

					//无法直接cout unsigned char(范围: 0～255) 类型的变量，要转换成int型
					cout << "[ " << int(*pixel) << ", " << int(*(pixel + 1)) << ", " << int(*(pixel + 2)) << "]   ";
					p = p + 1;
					if ((p % 5) == 0)	//每5个一换行
					{
						cout << endl;
					}
				}
			}
		}
	}


	//修改原像素
	p = 0;
	for (int k = 0; k < dims[2]; k++)
	{
		for (int j = 0; j < dims[1]; j++)
		{
			for (int i = 0; i < dims[0]; i++)
			{
				if (i > 0 && i < 501 && j > 0 && j < 501)    //500 * 500个像素,图片左下角
				{
					unsigned char* pixel = (unsigned char*)(reader->GetOutput()->GetScalarPointer(i, j, k));
					*pixel = 255;			      //red分量： 新像素值改为255
					*(pixel + 1) = 255 - *(pixel + 1);    //green分量： 新像素值用255减去原来像素值
					*(pixel + 2) = 255 - *(pixel + 2);    //blue分量： 新像素值用255减去原来像素值
				}
			}
		}
	}
	/*vtkSmartPointer<vtkTIFFWriter> tiffWriter = vtkSmartPointer<vtkTIFFWriter>::New();
	tiffWriter->SetFileName("1.tiff");
	tiffWriter->SetInputConnection();
	tiffWriter->Write();*/


	vtkSmartPointer<vtkImageShiftScale> ShiftScale = vtkSmartPointer<vtkImageShiftScale>::New();
	ShiftScale->SetInputConnection(reader->GetOutputPort());
	ShiftScale->SetOutputScalarTypeToUnsignedChar();
	ShiftScale->SetShift(1);
	ShiftScale->SetScale(127.5);
	ShiftScale->Update();
	auto a = ShiftScale->GetOutputPort();
}
