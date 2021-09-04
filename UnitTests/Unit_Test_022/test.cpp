#include "pch.h"
#include "TextureLoader.h"
#include "TextureSaver.h"
#include "Image.h"

using namespace hiveObliquePhotography;

//测试用例列表：
// *DeathTest_LoadMismatchingChannelTexture:载入通道不匹配图片
// *DeathTest_LoadNonexistTexture:载入不存在图片
// *SaveNonexistentTextureFile:保存纹理贴图不存在的文件名

const std::string MismatchingChannelTexturePath = TESTMODEL_DIR + std::string("Text022_Model/FullMask.png");
const std::string NonexistentTexture4LoadPath = TESTMODEL_DIR + std::string("Text022_Model/Mask.png");
const std::string NonexistentTexture4SavePath = TESTMODEL_DIR + std::string("Text022_Model/Mask.png");

TEST(Test_TextureLoader, DeathTest_LoadMismatchingChannelTexture)
{
	CImage<Eigen::Vector3i> TextureData1;
	auto* pTextureLoader = hiveDesignPattern::hiveCreateProduct<hiveObliquePhotography::ITextureLoader>(hiveUtility::hiveGetFileSuffix(MismatchingChannelTexturePath));
	pTextureLoader->loadTextureDataFromFileV(TextureData1, MismatchingChannelTexturePath);
	GTEST_ASSERT_EQ(TextureData1.getHeight(), 192);
}

TEST(Test_TextureLoader, DeathTest_LoadNonexistTexture)
{
	CImage<Eigen::Vector3i> TextureData2;
	auto* pTextureLoader = hiveDesignPattern::hiveCreateProduct<hiveObliquePhotography::ITextureLoader>(hiveUtility::hiveGetFileSuffix(NonexistentTexture4LoadPath));
	pTextureLoader->loadTextureDataFromFileV(TextureData2, NonexistentTexture4LoadPath);
	GTEST_ASSERT_EQ(TextureData2.getHeight(), 192);
}

TEST(Test_TextureSaver, SaveNonexistentTextureFile)
{
	CImage<Eigen::Vector3i> *TextureData3;
	auto* pTextureSaver = hiveDesignPattern::hiveCreateProduct<ITextureSaver>(hiveUtility::hiveGetFileSuffix(NonexistentTexture4SavePath));
	pTextureSaver->saveDate2FileV(*TextureData3, NonexistentTexture4SavePath);
	ASSERT_EQ(TextureData3, nullptr);
}

