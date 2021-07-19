#include "pch.h"
#include "PointCloudRetouchInterface.h"
#include "NeighborhoodBuilder.h"
#include "PointCloudRetouchConfig.h"
#include "PointCloudRetouchManager.h"
#include "PointLabelSet.h"

//	测试用例列表：
//	* Illegal_Input_Test：测试输入非法时能否正确报错，报错后能否继续正确运行
//	* Symmetry_Test：邻居关系具有对称性，即若A是B的邻居，则B一定是A的邻居 FIXME: 邻居关系似乎、好像、也许，并没有对称性 2021.7.12更新：K近邻没有对称性，半径搜索有对称性，此处使用半径搜索
//	* Reflexive_Test：邻居关系具有自反性，即A一定是A的邻居
//	* 暂时只测试INeighborhoodBuilder基类的功能

using namespace hiveObliquePhotography::PointCloudRetouch;

const auto Signature = KEYWORD::EUCLIDEAN_NEIGHBOR_BUILDER;

class CTestNeighborhoodBuilder :public testing::Test
{
public:
	const std::string RadiusConfigFilePath = TESTMODEL_DIR + std::string("Config/Test016_PointCloudRetouchConfig_RADIUS.xml");
	const std::string NearestConfigFilePath = TESTMODEL_DIR + std::string("Config/Test016_PointCloudRetouchConfig_NEAREST.xml");

protected:
	void SetUp() override
	{}

	void TearDown() override
	{}

	CPointCloudRetouchManager* generateRandomTestee(int vPointNum, int vFrom, int vTo, const std::string& vConfigFilePath)
	{
		PointCloud_t::Ptr pCloud(new PointCloud_t);
		for (int i = 0; i < vPointNum; i++)
		{
			PointCloud_t::PointType Point;
			Point.x = hiveMath::hiveGenerateRandomInteger(vFrom, vTo);
			Point.y = hiveMath::hiveGenerateRandomInteger(vFrom, vTo);
			Point.z = hiveMath::hiveGenerateRandomInteger(vFrom, vTo);
			pCloud->push_back(Point);
		}

		hiveConfig::CHiveConfig* pConfig = new CPointCloudRetouchConfig;
		hiveConfig::hiveParseConfig(vConfigFilePath, hiveConfig::EConfigType::XML, pConfig);

		auto pManager = CPointCloudRetouchManager::getInstance();
		pManager->init(pCloud, pConfig->findSubconfigByName("Retouch"));

		return pManager;
	}
};

TEST_F(CTestNeighborhoodBuilder, Radius_Illegal_Input_Test)
{
	auto pManagerRadius = generateRandomTestee(1000, -100, 100, NearestConfigFilePath);

	ASSERT_NO_THROW(pManagerRadius->buildNeighborhood(50, 1));
	ASSERT_ANY_THROW(pManagerRadius->buildNeighborhood(-1, 1));
	ASSERT_ANY_THROW(pManagerRadius->buildNeighborhood(1000000, 1));
}

TEST_F(CTestNeighborhoodBuilder, Nearest_Illegal_Input_Test)
{
	auto pManagerNearest = generateRandomTestee(1000, -100, 100, NearestConfigFilePath);

	ASSERT_NO_THROW(pManagerNearest->buildNeighborhood(50, 1));
	ASSERT_ANY_THROW(pManagerNearest->buildNeighborhood(-1, 1));
	ASSERT_ANY_THROW(pManagerNearest->buildNeighborhood(1000000, 1));
}

TEST_F(CTestNeighborhoodBuilder, Radius_Symmetry_Test)
{
	auto pManager = generateRandomTestee(1000, -100, 100, RadiusConfigFilePath);
	int SeedClusterIndex = 0;

	for (size_t i = 0; i < 500; i++)
	{
		auto TestIndex = hiveMath::hiveGenerateRandomInteger(0, 999);
		
		auto TestIndexNeighborhood = pManager->buildNeighborhood(TestIndex, ++SeedClusterIndex);
		if (TestIndexNeighborhood.size() == 0)
			continue;

		auto Neighbor = TestIndexNeighborhood.back();
		
		auto Neighborhood = pManager->buildNeighborhood(Neighbor, ++SeedClusterIndex);
		auto k = Neighborhood.begin();
		for (; k != Neighborhood.end(); ++k)
			if (*k == TestIndex)
				break;

		ASSERT_NE(k, Neighborhood.end());
	}
}