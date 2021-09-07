#include "pch.h"
#include "Image.h"

using hiveObliquePhotography::CColorTraits;

//注意大多数失败的测试用例可以在编译期检测出来，因此无法通过编译
//这里将这些测试用例注释

//测试用例列表：
//  * GeneralTypeTest: 能够正确解析常见颜色类型
//  * UnboundedTypeTest: 尝试解析变长的颜色类型，通道数会返回0
//  * StructTypeTest: 尝试解析结构体类的颜色类型
//  * DerivedTypeTest: 尝试解析派生的颜色类型，如cv、&*
//  * IllegalTypeTest: 尝试解析其他非法的的颜色类型，如void、enum

template <class ColorType, class ScalarType, size_t ChannelNum>
void EXPECT_COLOR_TRAITS(int vLine)
{
	EXPECT_EQ(typeid(CColorTraits<ColorType>::Scalar_t).hash_code(), typeid(ScalarType).hash_code());
	if (typeid(CColorTraits<ColorType>::Scalar_t).hash_code() != typeid(ScalarType).hash_code())
		std::cout << "Line " << vLine << ": " << typeid(ColorType).name() << std::endl;

	EXPECT_EQ(CColorTraits<ColorType>::extractChannel(), ChannelNum);
	if (CColorTraits<ColorType>::extractChannel() != ChannelNum)
		std::cout << "Line " << vLine << ": " << typeid(ColorType).name() << std::endl;
}

TEST(Test_ColorTraits, GeneralTypeTest)
{
	EXPECT_COLOR_TRAITS<int, int, 1>(__LINE__);
	EXPECT_COLOR_TRAITS<int[3], int, 3>(__LINE__);
	EXPECT_COLOR_TRAITS<std::array<int, 3>, int, 3>(__LINE__);
	EXPECT_COLOR_TRAITS<Eigen::Vector3i, int, 3>(__LINE__);
	EXPECT_COLOR_TRAITS<Eigen::RowVector3i, int, 3>(__LINE__);
}

TEST(Test_ColorTraits, UnboundedTypeTest)
{
	//EXPECT_COLOR_TRAITS<int[], int, 0>(__LINE__);
	EXPECT_COLOR_TRAITS<std::vector<int>, int, 0>(__LINE__);
	EXPECT_COLOR_TRAITS<Eigen::VectorXi, int, 0>(__LINE__);
}

struct SRgb
{
	using value_type = int;
	int R, G, B;
};
struct SRgbFloat
{
	using value_type = int;
	int R, G, B;
	float A;
};
struct SRgbNoValueType
{
	int R, G, B;
};

struct SRgbWrongValueType
{
	using value_type = double;
	int R, G, B;
};
struct SRgbVariablelValueType
{
	int R, G, B, value_type;
};
struct SRgbVoidValueType
{
	using value_type = void;
	int R, G, B;
};
struct SVoid
{
	using value_type = int;
};

TEST(Test_ColorTraits, StructTypeTest)
{
	EXPECT_COLOR_TRAITS<SRgb, int, 3>(__LINE__);
	//EXPECT_COLOR_TRAITS<SRgbFloat, int, 0>(__LINE__);
	//EXPECT_COLOR_TRAITS<SRgbNoValueType, void, 0>(__LINE__);
	//EXPECT_COLOR_TRAITS<SRgbWrongValueType, double, 0>(__LINE__); //TODO：暂时无法解决这个问题
	//EXPECT_COLOR_TRAITS<SRgbVariablelValueType, void, 0>(__LINE__);
	//EXPECT_COLOR_TRAITS<SRgbVoidValueType, void, 0>(__LINE__);
	//EXPECT_COLOR_TRAITS<SVoid, int, 0>(__LINE__);
}

TEST(Test_ColorTraits, DerivedTypeTest)
{
	//EXPECT_COLOR_TRAITS<SRgb&, int, 0>(__LINE__);
	//EXPECT_COLOR_TRAITS<SRgb&&, int, 0>(__LINE__);
	//EXPECT_COLOR_TRAITS<SRgb*, void, 0>(__LINE__);
	EXPECT_COLOR_TRAITS<const SRgb, int, 3>(__LINE__);
	//EXPECT_COLOR_TRAITS<volatile SRgb, void, 0>(__LINE__);
	//EXPECT_COLOR_TRAITS<const volatile SRgb, void, 0>(__LINE__);
}

enum EEnum
{
	R, G, B
};
enum class EEnumClass
{
	R, G, B
};

TEST(Test_ColorTraits, IllegalTypeTest)
{
	//EXPECT_COLOR_TRAITS<void, void, 0>(__LINE__);
	//EXPECT_COLOR_TRAITS<EEnum, void, 0>(__LINE__);
	//EXPECT_COLOR_TRAITS<EEnumClass, void, 0>(__LINE__);
}