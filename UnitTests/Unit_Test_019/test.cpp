#include "pch.h"
#include "TextureSynthesizer.h"
#include "boost/archive/text_iarchive.hpp"
#include "boost/archive/text_oarchive.hpp"
#include "boost/serialization/vector.hpp"

#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

//测试用例列表：
//  * DeathTest_EmptyInput: 输入为空，期待抛出异常
//  * DeathTest_DifferentSizesOfMaskAndScene: 输入大小不同的Mask和Scene，期待抛出异常
//  * AllBlaskMask: Mask全为0，期望Output和Scene一样
//  * SquareMask: 从图片中读取Mask，该图片的边缘值为255，即Mask为规则的正方形，边缘为0，输出保存为图片
//  * RandomMask：从图片中读取Mask，该图片的白色区域不规则，输出保存为图片

using namespace hiveObliquePhotography::PointCloudRetouch;

const auto InputImagePath = TESTMODEL_DIR + std::string("Test019_Model/input.png");
const auto SceneImagePath = TESTMODEL_DIR + std::string("Test019_Model/scene.png");
const auto MaskImagePath = TESTMODEL_DIR + std::string("Test019_Model/Mask.png");
const auto RandomMaskImagePath = TESTMODEL_DIR + std::string("Test019_Model/RandomMask.png");
const auto AllBlackMaskResultImagePath = TESTMODEL_DIR + std::string("Test019_Model/AllBlackMaskResultImage.png");
const auto SquareMaskResultImagePath = TESTMODEL_DIR + std::string("Test019_Model/SquareMaskResultImage.png");
const auto RandomMaskResultImagePath = TESTMODEL_DIR + std::string("Test019_Model/RandomMaskResultImage.png");
const auto HeightInputImagePath = TESTMODEL_DIR + std::string("Test019_Model/inputH.png");
const auto HeightMaskImagePath = TESTMODEL_DIR + std::string("Test019_Model/maskH.png");
const auto HeightSceneImagePath = TESTMODEL_DIR + std::string("Test019_Model/sceneH.png");
const auto HeightResultImagePath = TESTMODEL_DIR + std::string("Test019_Model/ResultH.png");

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

	void ReadImage(const std::string& vImagePath, Eigen::Matrix<Eigen::Vector3i, -1, -1>& voTexture)
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

	void ReadMask(const std::string& vImagePath, Eigen::MatrixXi& voMask)
	{
		const char* filepath = vImagePath.c_str();
		int Width, Height, BytesPerPixel;
		unsigned char* ImageData = stbi_load(filepath, &Width, &Height, &BytesPerPixel, 0);

		voMask.resize(Height, Width);
		for (int i = 0; i < Height; i++)
			for (int k = 0; k < Width; k++)
				voMask(i, k) = ImageData[(i * Width + k) * BytesPerPixel] / 255;
	}

	void GenerateMask(Eigen::MatrixXi& voMask, int vMode)
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

	void GenerateImage(Eigen::Matrix<Eigen::Vector3i, -1, -1>& voTexture, Eigen::Vector3i vMode)
	{
		for (int i = 0; i < voTexture.rows(); i++)
			for (int k = 0; k < voTexture.cols(); k++)
			{
				if (vMode.x() < 0 || vMode.y() < 0 || vMode.z() < 0)
					voTexture(i, k) = { hiveMath::hiveGenerateRandomInteger(0, 255), hiveMath::hiveGenerateRandomInteger(0, 255), hiveMath::hiveGenerateRandomInteger(0, 255) };
				else
					voTexture(i, k) = vMode;
			}
	}

	void GenerateResultImage(const Eigen::Matrix<Eigen::Vector3i, -1, -1>& vTexture, const std::string& vOutputImagePath)
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

};

TEST_F(TestTextureSynthesizer, DeathTest_EmptyInput)
{
	Eigen::Matrix<Eigen::Vector3i, -1, -1> InputTexture;
	Eigen::Matrix<Eigen::Vector3i, -1, -1> OutputTexture;
	Eigen::MatrixXi MaskTexture;

	CTextureSynthesizer<int, 3> TextureSynthesizer;
	EXPECT_ANY_THROW(TextureSynthesizer.execute(InputTexture, MaskTexture, OutputTexture));
}

TEST_F(TestTextureSynthesizer, DeathTest_DifferentSizesOfMaskAndScene)
{
	Eigen::Matrix<Eigen::Vector3i, -1, -1> InputTexture;
	Eigen::Matrix<Eigen::Vector3i, -1, -1> OutputTexture;

	ReadImage(InputImagePath, InputTexture);
	ReadImage(SceneImagePath, OutputTexture);

	Eigen::MatrixXi MaskTexture(OutputTexture.rows() - 1, OutputTexture.cols() - 1);
	GenerateMask(MaskTexture, 0);

	CTextureSynthesizer<int, 3> TextureSynthesizer;
	EXPECT_ANY_THROW(TextureSynthesizer.execute(InputTexture, MaskTexture, OutputTexture));
}

TEST_F(TestTextureSynthesizer, AllBlackMask)
{
	Eigen::Matrix<Eigen::Vector3i, -1, -1> InputTexture;
	Eigen::Matrix<Eigen::Vector3i, -1, -1> OutputTexture;

	ReadImage(InputImagePath, InputTexture);
	ReadImage(SceneImagePath, OutputTexture);

	Eigen::MatrixXi MaskTexture(OutputTexture.rows(), OutputTexture.cols());
	GenerateMask(MaskTexture, 0);

	Eigen::Matrix<Eigen::Vector3i, -1, -1> SceneTexture = OutputTexture;

	CTextureSynthesizer<int, 3> TextureSynthesizer;
	TextureSynthesizer.execute(InputTexture, MaskTexture, OutputTexture);

	for (int i = 0; i < OutputTexture.rows(); i++)
		for (int k = 0; k < OutputTexture.cols(); k++)
			EXPECT_EQ(OutputTexture(i, k), SceneTexture(i, k));
}

TEST_F(TestTextureSynthesizer, SquareMask)
{
	Eigen::Matrix<Eigen::Vector3i, -1, -1> InputTexture;
	Eigen::Matrix<Eigen::Vector3i, -1, -1> OutputTexture;

	ReadImage(InputImagePath, InputTexture);
	ReadImage(SceneImagePath, OutputTexture);

	Eigen::MatrixXi MaskTexture(OutputTexture.rows(), OutputTexture.cols());
	/*GenerateMask(MaskTexture, -1);*/
	ReadMask(MaskImagePath, MaskTexture);

	CTextureSynthesizer<int, 3> TextureSynthesizer;
	TextureSynthesizer.execute(InputTexture, MaskTexture, OutputTexture);

	GenerateResultImage(OutputTexture, SquareMaskResultImagePath);
}

TEST_F(TestTextureSynthesizer, RandomMask)
{
	Eigen::Matrix<Eigen::Vector3i, -1, -1> InputTexture;
	Eigen::Matrix<Eigen::Vector3i, -1, -1> OutputTexture;

	ReadImage(InputImagePath, InputTexture);
	ReadImage(SceneImagePath, OutputTexture);

	Eigen::MatrixXi MaskTexture(OutputTexture.rows(), OutputTexture.cols());
	ReadMask(RandomMaskImagePath, MaskTexture);

	CTextureSynthesizer<int, 3> TextureSynthesizer;
	TextureSynthesizer.execute(InputTexture, MaskTexture, OutputTexture);

	GenerateResultImage(OutputTexture, RandomMaskResultImagePath);
}

TEST_F(TestTextureSynthesizer, Height)
{
	Eigen::Matrix<Eigen::Vector3i, -1, -1> InputTexture;
	Eigen::Matrix<Eigen::Vector3i, -1, -1> OutputTexture;

	ReadImage(HeightInputImagePath, InputTexture);
	ReadImage(HeightSceneImagePath, OutputTexture);

	Eigen::MatrixXi MaskTexture(OutputTexture.rows(), OutputTexture.cols());
	ReadMask(HeightMaskImagePath, MaskTexture);

	CTextureSynthesizer<int, 3> TextureSynthesizer;
	TextureSynthesizer.execute(InputTexture, MaskTexture, OutputTexture);

	GenerateResultImage(OutputTexture, HeightResultImagePath);
}