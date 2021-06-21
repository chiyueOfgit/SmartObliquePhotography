#include "pch.h"
#include "PointCloudRetouchInterface.h"
#include "NeighborhoodBuilder.h"
#include "PointCloudRetouchManager.h"
#include "PointLabelSet.h"

//	测试用例列表：
//	* Illegal_Input_Test：测试输入非法时能否正确报错，报错后能否继续正确运行
//	* Symmetry_Test：邻居关系具有对称性，即若A是B的邻居，则B一定是A的邻居
//	* Anti_Reflexive_Test：邻居关系具有反自反性，即A一定不是A的邻居
//	* 暂时只测试INeighborhoodBuilder基类的功能

using namespace hiveObliquePhotography::PointCloudRetouch;

const auto Signature = KEYWORD::EUCLIDEAN_NEIGHBOR_BUILDER;

INeighborhoodBuilder* generateRandomTestee(int vPointNum, int vFrom, int vTo)
{
	PointCloud_t::Ptr pScene(new PointCloud_t);
	CPointLabelSet PointLabelSet;

	hiveConfig::CHiveConfig Config;
	CPointCloudRetouchManager::getInstance()->init(pScene, &Config);
	
	for (int i = 0; i < vPointNum; i++)
	{
		PointCloud_t::PointType Point;
		Point.x = hiveMath::hiveGenerateRandomInteger(vFrom, vTo);
		Point.y = hiveMath::hiveGenerateRandomInteger(vFrom, vTo);
		Point.z = hiveMath::hiveGenerateRandomInteger(vFrom, vTo);
		pScene->push_back(Point);
	}
	PointLabelSet.init(pScene->size());
	return hiveDesignPattern::hiveCreateProduct<INeighborhoodBuilder>(Signature, pScene, &PointLabelSet);
}


TEST(TestNeighborhoodBuilder, Illegal_Input_Test)
{
	auto pNeighborhoodBuilder = generateRandomTestee(1000, -100, 100);
	/*ASSERT_NO_THROW(pNeighborhoodBuilder = hiveDesignPattern::hiveGetOrCreateProduct<INeighborhoodBuilder>(Signature, pScene, &PointLabelSet));*/

	std::vector<pcl::index_t> Neighborhood;
	ASSERT_ANY_THROW(pNeighborhoodBuilder->buildNeighborhood(-1, -1, Neighborhood));
	ASSERT_ANY_THROW(pNeighborhoodBuilder->buildNeighborhood(999, 999, Neighborhood));
	ASSERT_ANY_THROW(pNeighborhoodBuilder->buildNeighborhood(50, 50, Neighborhood));
}

TEST(TestNeighborhoodBuilder, Symmetry_Test)
{
	auto pNeighborhoodBuilder = generateRandomTestee(1000, -100, 100);

	for (size_t i = 0; i < 500; i++)
	{
		auto TestIndex = hiveMath::hiveGenerateRandomInteger(0, 999);

		std::vector<pcl::index_t> Neighborhood;
		pNeighborhoodBuilder->buildNeighborhood(TestIndex, 0, Neighborhood);
		auto Neighbor = Neighborhood.front();

		pNeighborhoodBuilder->buildNeighborhood(Neighbor, 0, Neighborhood);
		auto k = Neighborhood.begin();
		for (; k != Neighborhood.end(); k++)
			if (*k == TestIndex)
				break;

		ASSERT_NE(k, Neighborhood.end());
	}
}

TEST(TestNeighborhoodBuilder, Anti_Reflexive_Test)
{
	auto pNeighborhoodBuilder = generateRandomTestee(1000, -100, 100);

	for (size_t i = 0; i < 500; i++)
	{
		auto TestIndex = hiveMath::hiveGenerateRandomInteger(0, 999);

		std::vector<pcl::index_t> Neighborhood;
		pNeighborhoodBuilder->buildNeighborhood(TestIndex, 0, Neighborhood);

		auto k = Neighborhood.begin();
		for (; k != Neighborhood.end(); k++)
			if (*k == TestIndex)
				break;

		ASSERT_EQ(k, Neighborhood.end());
	}
}
