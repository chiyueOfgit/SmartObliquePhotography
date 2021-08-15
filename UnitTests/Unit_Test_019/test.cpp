#include "pch.h"
#include "TextureSynthesizer.h"
#include "OrderIndependentTextureSynthesizer.h"
#include "MultithreadTextureSynthesizer.h"
#include "MipmapGenerator.h"
#include "boost/archive/text_iarchive.hpp"
#include "boost/archive/text_oarchive.hpp"
#include "boost/serialization/vector.hpp"

#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <common/CpuTimer.h>

#include "stb_image_write.h"

//测试用例列表：
//  * DeathTest_EmptyInput: 输入为空，期待抛出异常
//  * DeathTest_DifferentSizesOfMaskAndScene: 输入大小不同的Mask和Scene，期待抛出异常
//  * AllBlaskMask: Mask全为0，期望Output和Scene一样
//  * SquareMask: 从图片中读取Mask，该图片的边缘值为255，即Mask为规则的正方形，边缘为0，输出保存为图片
//  * RandomMask: 从图片中读取Mask，该图片的白色区域不规则，输出保存为图片
//  * SpecialInput: 特殊的Input情况
//  * Height: 补全高度信息的纹理并可视化
//  * GenerateMipmap: 生成输入纹理的mipmap

using namespace hiveObliquePhotography::PointCloudRetouch;

const auto InputImagePath = TESTMODEL_DIR + std::string("Test019_Model/input.png");
const auto SceneImagePath = TESTMODEL_DIR + std::string("Test019_Model/scene.png");
const auto MaskImagePath = TESTMODEL_DIR + std::string("Test019_Model/Mask.png");
const auto RandomMaskImagePath = TESTMODEL_DIR + std::string("Test019_Model/RandomMask.png");
const auto HoleMaskImagePath = TESTMODEL_DIR + std::string("Test019_Model/HoleMask.png");
const auto AllBlackMaskResultImagePath = TESTMODEL_DIR + std::string("Test019_Model/AllBlackMaskResultImage.png");
const auto SquareMaskResultImagePath = TESTMODEL_DIR + std::string("Test019_Model/SquareMaskResultImage.png");
const auto RandomMaskResultImagePath = TESTMODEL_DIR + std::string("Test019_Model/RandomMaskResultImage.png");
const auto HeightInputImagePath = TESTMODEL_DIR + std::string("Test019_Model/inputH.png");
const auto HeightMaskImagePath = TESTMODEL_DIR + std::string("Test019_Model/maskH.png");
const auto HeightSceneImagePath = TESTMODEL_DIR + std::string("Test019_Model/sceneH.png");
const auto HeightResultImagePath = TESTMODEL_DIR + std::string("Test019_Model/ResultH.png");

const std::vector<std::string> SpecialImageName{ /*"Diamond", "FourColor", */"LineColor" ,"Tangram", "Star"};
const auto SpecialImagePath = TESTMODEL_DIR + std::string("Test019_Model/SpecialTest/");

class TestTextureSynthesizer : public testing::Test
{
public:


protected:
	void SetUp() override
	{
	}

	void TearDown() override
	{

	}

	void _readImage(const std::string& vImagePath, Eigen::Matrix<Eigen::Vector3i, -1, -1>& voTexture)
	{
		const char* filepath = vImagePath.c_str();
		int Width, Height, BytesPerPixel;
		unsigned char* ImageData = stbi_load(filepath, &Width, &Height, &BytesPerPixel, 0);

		voTexture.resize(Height, Width);
		for (int i = 0; i < Height; i++)
			for (int k = 0; k < Width; k++)
				for (int Offset = 0; Offset < 3; Offset++)
					voTexture(i, k)[Offset] = ImageData[(i * Width + k) * BytesPerPixel + Offset];
	}

	void _readMask(const std::string& vImagePath, Eigen::MatrixXi& voMask)
	{
		const char* filepath = vImagePath.c_str();
		int Width, Height, BytesPerPixel;
		unsigned char* ImageData = stbi_load(filepath, &Width, &Height, &BytesPerPixel, 0);

		voMask.resize(Height, Width);
		for (int i = 0; i < Height; i++)
			for (int k = 0; k < Width; k++)
				voMask(i, k) = ImageData[(i * Width + k) * BytesPerPixel] / 255;
	}

	void _changeChannel(Eigen::Matrix<Eigen::Vector3i, -1, -1>& vioTexture, int vMode)
	{
		for (int i = 0; i < vioTexture.rows(); i++)
			for (int k = 0; k < vioTexture.cols(); k++)
			{
				auto Pixel = vioTexture(i, k);
				if (!Pixel[0] && Pixel[1] && vMode == 0)
				{
					Pixel[0] = -Pixel[1];
					Pixel[1] = 0;
				}
				else if (Pixel[0] < 0 && vMode == 1)
				{
					Pixel[1] = -Pixel[0];
					Pixel[0] = 0;
				}
			}
	}

	void _generateMask(Eigen::MatrixXi& voMask, int vMode)
	{
		for (int i = 0; i < voMask.rows(); i++)
			for (int k = 0; k < voMask.cols(); k++)
			{
				if (vMode == -1)
					voMask(i, k) = hiveMath::hiveGenerateRandomInteger(0, 1);
				else
					voMask(i, k) = vMode;
			}
	}

	void _generateResultImage(const Eigen::Matrix<Eigen::Vector3i, -1, -1>& vTexture, const std::string& vOutputImagePath)
	{
		const auto Width = vTexture.cols();
		const auto Height = vTexture.rows();
		const auto BytesPerPixel = 3;
		auto ResultImage = new unsigned char[Width * Height * BytesPerPixel];
		for (auto i = 0; i < Height; i++)
			for (auto k = 0; k < Width; k++)
			{
				auto Offset = (i * Width + k) * BytesPerPixel;
				ResultImage[Offset] = vTexture.coeff(i, k)[0];
				ResultImage[Offset + 1] = vTexture.coeff(i, k)[1];
				ResultImage[Offset + 2] = vTexture.coeff(i, k)[2];
			}

		stbi_write_png(vOutputImagePath.c_str(), Width, Height, BytesPerPixel, ResultImage, 0);
		stbi_image_free(ResultImage);
	}

	Eigen::Matrix<Eigen::Vector3i, -1, -1> _getMipMap(const Eigen::Matrix<Eigen::Vector3i, -1, -1>& vTexture)
	{
		Eigen::Matrix<Eigen::Vector3i, -1, -1> Mipmap((vTexture.rows() + 1) / 2, (vTexture.cols() + 1) / 2);
		_gaussianBlur(vTexture, Mipmap);
		return Mipmap;
	}

	void _gaussianBlur(const Eigen::Matrix<Eigen::Vector3i, -1, -1>& vTexture, Eigen::Matrix<Eigen::Vector3i, -1, -1>& voMipmap)
	{
		auto GaussianKernal = _getGaussianKernal(3, 0);
		for (int i = 0; i < voMipmap.rows(); i++)
			for (int k = 0; k < voMipmap.cols(); k++)
				for (int Channel = 0; Channel < 3; Channel++)
					voMipmap(i, k)[Channel] = _gaussianFilter(vTexture, GaussianKernal, Channel, i * 2, k * 2);
	}

	float _gaussianFilter(const Eigen::Matrix<Eigen::Vector3i, -1, -1>& vTexture, const Eigen::Matrix<float, -1, -1>& vGaussianKernal, int vChannel, int vRow, int vCol)
	{
		float Value = 0.0;
		int GaussianKernalRowIndex = -1;
		int GaussianKernalColIndex = -1;
		for (int i = vRow - (vGaussianKernal.rows() - 1) / 2; i <= vRow + (vGaussianKernal.rows() - 1) / 2; i++)
		{
			GaussianKernalRowIndex++;
			for (int k = vCol - (vGaussianKernal.cols() - 1) / 2; k <= vCol + (vGaussianKernal.cols() - 1) / 2; k++)
			{
				GaussianKernalColIndex++;
				if (i < 0 || k < 0 || i >= vTexture.rows() || k >= vTexture.cols())
					continue;
				Value += vGaussianKernal.coeff(GaussianKernalRowIndex, GaussianKernalColIndex) * vTexture.coeff(i, k)[vChannel];
			}
			GaussianKernalColIndex = -1;
		}

		return Value;
	}

	Eigen::Matrix<float, -1, -1> _getGaussianKernal(int vKernalSize, float vSigma)
	{
		if (!vSigma)
			vSigma = 0.3 * ((vKernalSize - 1) * 0.5 - 1) + 0.8;

		float SumWeight = 0.0;
		Eigen::Matrix<float, -1, -1> GaussianKernal(vKernalSize, vKernalSize);

		for (int i = 0; i < vKernalSize; i++)
			for (int k = 0; k < vKernalSize; k++)
			{
				GaussianKernal(i, k) = _getGaussianWeight(std::abs((vKernalSize - 1) / 2 - i) + std::abs((vKernalSize - 1) / 2 - k), vSigma);
				SumWeight += GaussianKernal.coeff(i, k);
			}

		GaussianKernal /= SumWeight;

		return GaussianKernal;
	}

	float _getGaussianWeight(float vRadius, float vSigma)
	{
		// Considering that normalization is required later, the coefficient is not calculated
		return std::pow(std::numbers::e, -vRadius * vRadius / (2 * vSigma * vSigma));
	}
};

//TEST_F(TestTextureSynthesizer, DeathTest_EmptyInput)
//{
//	Eigen::Matrix<Eigen::Vector3i, -1, -1> InputTexture;
//	Eigen::Matrix<Eigen::Vector3i, -1, -1> OutputTexture;
//	Eigen::MatrixXi MaskTexture;
//
//	CTextureSynthesizer<int, 3> TextureSynthesizer;
//	EXPECT_ANY_THROW(TextureSynthesizer.execute(InputTexture, MaskTexture, OutputTexture));
//}

//TEST_F(TestTextureSynthesizer, DeathTest_DifferentSizesOfMaskAndScene)
//{
//	Eigen::Matrix<Eigen::Vector3i, -1, -1> InputTexture;
//	Eigen::Matrix<Eigen::Vector3i, -1, -1> OutputTexture;
//
//	_readImage(InputImagePath, InputTexture);
//	_readImage(SceneImagePath, OutputTexture);
//
//	Eigen::MatrixXi MaskTexture(OutputTexture.rows() - 1, OutputTexture.cols() - 1);
//	_generateMask(MaskTexture, 0);
//
//	CTextureSynthesizer<int, 3> TextureSynthesizer;
//	EXPECT_ANY_THROW(TextureSynthesizer.execute(InputTexture, MaskTexture, OutputTexture));
//}

//TEST_F(TestTextureSynthesizer, AllBlackMask)
//{
//	Eigen::Matrix<Eigen::Vector3i, -1, -1> InputTexture;
//	Eigen::Matrix<Eigen::Vector3i, -1, -1> OutputTexture;
//
//	_readImage(InputImagePath, InputTexture);
//	_readImage(SceneImagePath, OutputTexture);
//
//	Eigen::MatrixXi MaskTexture(OutputTexture.rows(), OutputTexture.cols());
//	_generateMask(MaskTexture, 0);
//
//	Eigen::Matrix<Eigen::Vector3i, -1, -1> SceneTexture = OutputTexture;
//
//	CTextureSynthesizer<int, 3> TextureSynthesizer;
//	TextureSynthesizer.execute(InputTexture, MaskTexture, OutputTexture);
//
//	for (int i = 0; i < OutputTexture.rows(); i++)
//		for (int k = 0; k < OutputTexture.cols(); k++)
//			EXPECT_EQ(OutputTexture(i, k), SceneTexture(i, k));
//}

TEST_F(TestTextureSynthesizer, SquareMask)
{
	Eigen::Matrix<Eigen::Vector3i, -1, -1> InputTexture;
	Eigen::Matrix<Eigen::Vector3i, -1, -1> OutputTexture;

	_readImage(InputImagePath, InputTexture);
	_readImage(SceneImagePath, OutputTexture);

	Eigen::MatrixXi MaskTexture(OutputTexture.rows(), OutputTexture.cols());
	/*_generateMask(MaskTexture, -1);*/
	_readMask(MaskImagePath, MaskTexture);
	
	hiveCommon::CCPUTimer Timer;
	Timer.start();
	CMultithreadTextureSynthesizer<int, 3> TextureSynthesizer;
	TextureSynthesizer.execute(InputTexture, MaskTexture, OutputTexture);
	Timer.stop();
	std::cout <<"RunTime: " << Timer.getElapsedTimeInMS() << std::endl;
	
	_generateResultImage(OutputTexture, SquareMaskResultImagePath);
}

//TEST_F(TestTextureSynthesizer, RandomMask)
//{
//	Eigen::Matrix<Eigen::Vector3i, -1, -1> InputTexture;
//	Eigen::Matrix<Eigen::Vector3i, -1, -1> OutputTexture;
//
//	_readImage(InputImagePath, InputTexture);
//	_readImage(SceneImagePath, OutputTexture);
//
//	Eigen::MatrixXi MaskTexture(OutputTexture.rows(), OutputTexture.cols());
//	_readMask(RandomMaskImagePath, MaskTexture);
//
//	CTextureSynthesizer<int, 3> TextureSynthesizer;
//	TextureSynthesizer.execute(InputTexture, MaskTexture, OutputTexture);
//
//	_generateResultImage(OutputTexture, RandomMaskResultImagePath);
//}

//TEST_F(TestTextureSynthesizer, SpecialInput)
//{
//	for(auto& Name: SpecialImageName)
//	{
//		Eigen::Matrix<Eigen::Vector3i, -1, -1> InputTexture;
//		Eigen::Matrix<Eigen::Vector3i, -1, -1> OutputTexture;
//
//		_readImage({ SpecialImagePath + Name + "Input.png" }, InputTexture);
//		_readImage({ SpecialImagePath + Name + ".png" }, OutputTexture);
//
//		Eigen::MatrixXi MaskTexture(OutputTexture.rows(), OutputTexture.cols());
//		/*_generateMask(MaskTexture, -1);*/
//		_readMask(MaskImagePath, MaskTexture);
//
//		CTextureSynthesizer<int, 3> TextureSynthesizer;
//		TextureSynthesizer.execute(InputTexture, MaskTexture, OutputTexture);
//
//		_generateResultImage(OutputTexture, { SpecialImagePath + Name + "Mask" + ".png"});
//	}
//	
//}

//TEST_F(TestTextureSynthesizer, Height)
//{
//	Eigen::Matrix<Eigen::Vector3i, -1, -1> InputTexture;
//	Eigen::Matrix<Eigen::Vector3i, -1, -1> OutputTexture;
//
//	_readImage(HeightInputImagePath, InputTexture);
//	_readImage(HeightSceneImagePath, OutputTexture);
//
//	_changeChannel(InputTexture, 0);
//	_changeChannel(OutputTexture, 0);
//
//	Eigen::MatrixXi MaskTexture(OutputTexture.rows(), OutputTexture.cols());
//	_readMask(HeightMaskImagePath, MaskTexture);
//
//	CTextureSynthesizer<int, 3> TextureSynthesizer;
//	TextureSynthesizer.execute(InputTexture, MaskTexture, OutputTexture);
//
//	_changeChannel(OutputTexture, 1);
//
//	_generateResultImage(OutputTexture, HeightResultImagePath);
//}

//TEST_F(TestTextureSynthesizer, GenerateMipmap)
//{
//	Eigen::Matrix<Eigen::Vector3i, -1, -1> InputTexture;
//	Eigen::Matrix<Eigen::Vector3i, -1, -1> MipmapTexture;
//
//	_readImage(InputImagePath, InputTexture);
//	CMipmapGenerator<Eigen::Vector3i> MipmapGenerator;
//	MipmapTexture = MipmapGenerator.getMipmap(InputTexture);
//	_generateResultImage(MipmapTexture, TESTMODEL_DIR + std::string("Test019_Model/mipmap2.png"));
//
//}
//
//TEST_F(TestTextureSynthesizer, GaussianBlur)
//{
//	Eigen::Matrix<Eigen::Vector3i, -1, -1> InputTexture;
//	Eigen::Matrix<Eigen::Vector3i, -1, -1> ResultTexture;
//
//	_readImage(InputImagePath, InputTexture);
//	CMipmapGenerator<Eigen::Vector3i> MipmapGenerator;
//	MipmapGenerator.setKernalSize(9);
//	ResultTexture = MipmapGenerator.executeGaussianBlur(InputTexture);
//	_generateResultImage(ResultTexture, TESTMODEL_DIR + std::string("Test019_Model/Gaussian.png"));
//}

//TEST_F(TestTextureSynthesizer, GaussianPyramid)
//{
//	Eigen::Matrix<Eigen::Vector3i, -1, -1> InputTexture;
//	std::vector<Eigen::Matrix<Eigen::Vector3i, -1, -1>> GaussianPyramid, GaussianStack;
//
//	_readImage(InputImagePath, InputTexture);
//	CMipmapGenerator<Eigen::Vector3i> MipmapGenerator;
//	MipmapGenerator.setKernalSize(20);
//	int Layer = 40;
//	GaussianPyramid = MipmapGenerator.getGaussianPyramid(InputTexture, Layer);
//	GaussianStack = MipmapGenerator.getGaussianStack(InputTexture, Layer);
//
//	for (int i = 0; i < Layer; i++)
//	{
//		_generateResultImage(GaussianPyramid[i], TESTMODEL_DIR + std::string("Test019_Model/GaussianPyramid/GaussianPyramid_") + std::to_string(i) + std::string(".png"));
//		_generateResultImage(GaussianStack[i], TESTMODEL_DIR + std::string("Test019_Model/GaussianStack/GaussianStack_") + std::to_string(i) + std::string(".png"));
//	}
//}

//TEST_F(TestTextureSynthesizer, MipmapWithMask)
//{
//	Eigen::Matrix<Eigen::Vector3i, -1, -1> OutputTexture;
//	_readImage(SceneImagePath, OutputTexture);
//
//	Eigen::MatrixXi MaskTexture(OutputTexture.rows(), OutputTexture.cols());
//	_readMask(MaskImagePath, MaskTexture);
//
//	CMipmapGenerator<Eigen::Vector3i> MipmapGenerator;
//	MipmapGenerator.setKernalSize(3);
//	auto Mipmap = MipmapGenerator.getMipmap(OutputTexture, MaskTexture);
//
//	for (int i = 0; i < Mipmap.rows(); i++)
//		for (int k = 0; k < Mipmap.cols(); k++)
//			if (Mipmap(i, k)[0] == -1)
//			{
//				std::cout << "(" << i << ", " << k << ")\n";
//				for (int m = 0; m < 3; m++)
//					Mipmap(i, k)[m] = 255;
//			}
//
//	_generateResultImage(Mipmap, SquareMaskResultImagePath);
//}

//TEST_F(TestTextureSynthesizer, GaussianPyramidWithMask)
//{
//	Eigen::Matrix<Eigen::Vector3i, -1, -1> InputTexture;
//	std::vector<Eigen::Matrix<Eigen::Vector3i, -1, -1>> GaussianPyramid, GaussianStack;
//	Eigen::MatrixXi MaskTexture(InputTexture.rows(), InputTexture.cols());
//	
//	_readMask(MaskImagePath, MaskTexture);
//	_readImage(SceneImagePath, InputTexture);
//	CMipmapGenerator<Eigen::Vector3i> MipmapGenerator;
//	MipmapGenerator.setKernalSize(9);
//	int Layer = 10;
//	GaussianPyramid = MipmapGenerator.getGaussianPyramid(InputTexture, Layer, MaskTexture);
//	GaussianStack = MipmapGenerator.getGaussianStack(InputTexture, Layer, MaskTexture);
//
//	for (int i = 0; i < Layer; i++)
//	{
//		_generateResultImage(GaussianPyramid[i], TESTMODEL_DIR + std::string("Test019_Model/GaussianPyramid/GaussianPyramid_") + std::to_string(i) + std::string(".png"));
//		_generateResultImage(GaussianStack[i], TESTMODEL_DIR + std::string("Test019_Model/GaussianStack/GaussianStack_") + std::to_string(i) + std::string(".png"));
//	}
//}