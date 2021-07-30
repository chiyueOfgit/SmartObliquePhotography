#include "pch.h"
#include "TextureSynthesizer.h"
#include "toojpeg.h"
#include "boost/archive/text_iarchive.hpp"
#include "boost/archive/text_oarchive.hpp"
#include "boost/serialization/vector.hpp"
#include <fstream>
#include <ostream>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

//测试用例列表：
//  * DeathTest_EmptyInput: 输入为空，期待抛出异常
//  * DeathTest_DifferentSizesOfMaskAndScene: 输入大小不同的Mask和Scene，期待抛出异常
//  * AreaNotHolesWontBeModified: 非洞的场景区域不会被修改
//  * RandomMask: 随机Mask
//  * RemainBoundaryMask: 保留边缘Mask

using namespace hiveObliquePhotography::PointCloudRetouch;

const auto InputFilePath = TESTMODEL_DIR + std::string("Test019_Model/input.txt");
const auto OutputFilePath = TESTMODEL_DIR + std::string("Test019_Model/output.txt");
const auto ResultImagePath = TESTMODEL_DIR + std::string("Test019_Model/ResultImage.png");
std::ofstream ResultJPG(ResultImagePath, std::ios_base::out | std::ios_base::binary);

class TestTextureSynthesizer : public testing::Test
{
public:
	std::vector<std::vector<Eigen::Vector3i>> m_InputImage;
	std::vector<std::vector<bool>> m_MaskImage;
	std::vector<std::vector<Eigen::Vector3i>> m_OutputImage;

protected:
	void SetUp() override
	{
		LoadImageTxt(InputFilePath, m_InputImage);
		GenerateMask(m_MaskImage, 192, 192);
		LoadImageTxt(OutputFilePath, m_OutputImage);
	}

	void TearDown() override
	{

	}

	void GenerateMask(std::vector<std::vector<bool>>& voMask, int vRow, int vCol)
	{
		for (int i = 0; i < vRow; i++)
		{
			std::vector<bool> TempRow;
			for (int k = 0; k < vCol; k++)
				TempRow.push_back(1);
				//TempRow.push_back(hiveMath::hiveGenerateRandomInteger(0, 1));
			voMask.push_back(TempRow);
		}
	}

	void LoadImageTxt(const std::string& vFilePath, std::vector<std::vector<Eigen::Vector3i>>& voImage)
	{
		std::ifstream File(vFilePath);
		std::vector<std::string> TextString;
		std::string Line = "";
		int LineCount = 1;
		std::vector<Eigen::Vector3i> ImageCol;
		Eigen::Vector3i Pixel;

		if (!File.is_open())
			return;

		while (std::getline(File, Line))
		{
			if (Line.size() > 0)
			{
				if (Line[0] != 'L')
				{
					if (LineCount == 1)
					{
						Pixel[0] = std::atoi(Line.c_str());
						LineCount++;
					}
					else if (LineCount == 2)
					{
						Pixel[1] = std::atoi(Line.c_str());
						LineCount++;
					}
					else if (LineCount == 3)
					{
						Pixel[2] = std::atoi(Line.c_str());
						ImageCol.push_back(Pixel);
						LineCount = 1;
					}
				}
				else
				{
					voImage.push_back(ImageCol);
					ImageCol.clear();
				}
			}
		}
	}

	void TransferVector2MatrixVec3i(const std::vector<std::vector<Eigen::Vector3i>>& vImage, Eigen::Matrix<Eigen::Vector3i, -1, -1>& voTexture)
	{
		for (int i = 0; i < vImage.size(); i++)
			for (int k = 0; k < vImage[i].size(); k++)
				for (int m = 0; m < 3; m++)
					voTexture(i, k)[m] = vImage[i][k][m];
	}

	void TransferVector2MatrixXi(const std::vector<std::vector<bool>>& vImage, Eigen::MatrixXi& voTexture)
	{
		for (int i = 0; i < vImage.size(); i++)
			for (int k = 0; k < vImage[i].size(); k++)
				voTexture(i, k) = vImage[i][k];
	}

};

void OutputFunc(unsigned char byte)
{
	ResultJPG << byte;
}

//TEST_F(TestTextureSynthesizer, RandomMask)
//{
//	EXPECT_EQ(m_InputImage.size(), 64);
//	EXPECT_EQ(m_OutputImage.size(), 192);
//
//	Eigen::Matrix<Eigen::Vector3i, -1, -1> InputTexture(m_InputImage.size(), m_InputImage[0].size());
//	Eigen::Matrix<Eigen::Vector3i, -1, -1> OutputTexture(m_OutputImage.size(), m_OutputImage[0].size());
//	Eigen::MatrixXi MaskTexture(m_MaskImage.size(), m_MaskImage[0].size());
//
//	TransferVector2MatrixVec3i(m_InputImage, InputTexture);
//	TransferVector2MatrixVec3i(m_OutputImage, OutputTexture);
//	TransferVector2MatrixXi(m_MaskImage, MaskTexture);
//
//	CTextureSynthesizer<Eigen::Vector3i> TextureSynthesizer;
//	TextureSynthesizer.execute(InputTexture, MaskTexture, OutputTexture);
//
//	// Write PPM
//	std::ofstream OutputPPM;
//	OutputPPM.open("Result.ppm");
//	OutputPPM << "P3\n" << m_OutputImage.size() << " " << m_OutputImage[0].size() << "\n255\n";
//	for (int i = 0; i < m_OutputImage.size(); i++)
//		for (int k = 0; k < m_OutputImage[0].size(); k++)
//			OutputPPM << OutputTexture.coeff(i, k)[0] << " " << OutputTexture.coeff(i, k)[1] << " " << OutputTexture.coeff(i, k)[2] << "\n";
//	OutputPPM.close();
//
//	const auto Width = m_OutputImage.size();
//	const auto Height = m_OutputImage[0].size();
//	const auto BytesPerPixel = 3;
//	auto ResultImage = new unsigned char[Width * Height * BytesPerPixel];
//	for (auto i = 0; i < Height; i++)
//		for (auto k = 0; k < Width; k++)
//		{
//			auto Offset = (i * Width + k) * BytesPerPixel;
//			ResultImage[Offset] = OutputTexture.coeff(i, k)[2];
//			ResultImage[Offset + 1] = OutputTexture.coeff(i, k)[1];
//			ResultImage[Offset + 2] = OutputTexture.coeff(i, k)[0];
//		}
//	const bool IsRGB = true;
//	const auto Quality = 90;
//	const bool Downsample = false;
//	const char* Comment = "Ramdom Mask Result Image";
//	
//	auto ok = TooJpeg::writeJpeg(OutputFunc, ResultImage, Width, Height, IsRGB, Quality, Downsample, Comment);
//	delete[] ResultImage;
//}

TEST()
{
	int w, h, n;
	//unsigned char* data = stbi_load("output.png", &w, &h, &n, 0);
	std::cout << ResultImagePath << std::endl;
	unsigned char* data = stbi_load("D:\\GithubDeskTopProjects\\Texture\\SmartObliquePhotography_TextureSynthesis\\UnitTests\\TestData\\Test019_Model/ResultImage.png", &w, &h, &n, 0);
	printf("%d, %d, %d\n", w, h, n);
}