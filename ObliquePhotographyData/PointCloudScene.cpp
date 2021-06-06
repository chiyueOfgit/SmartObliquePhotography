#include "pch.h"
#include "PointCloudScene.h"
#include <boost/format.hpp>
#include "common/UtilityInterface.h"
#include "PointCloudTile.h"
#include "PointCloudLoader.h"

using namespace hiveObliquePhotography;

CPointCloudScene::~CPointCloudScene()
{
	clear();
}

//*****************************************************************
//FUNCTION: 
pcl::PointCloud<pcl::PointSurfel>* CPointCloudScene::loadScene(const std::vector<std::string>& vFileNameSet)
{
	_ASSERTE(!vFileNameSet.empty());
//TODO：要保证vFileNameSet里面没有重复文件名

	clear();

	_ASSERTE(m_PointCloudScene.empty() && m_PointCloudTileMap.empty());

	for (const auto& e : vFileNameSet)
	{
		auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<IPointCloudLoader>(hiveUtility::hiveGetFileSuffix(e));
		if (pTileLoader)
		{
			pcl::PointCloud<pcl::PointSurfel>* pTile = pTileLoader->loadDataFromFile(e);
			m_PointCloudTileMap.insert(std::make_pair(e, new CPointCloudTile(pTile)));
			m_PointCloudScene += *pTile;
		}
		else
		{
			_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to load tile [%1%] due to unknown format.", e));
		}
	}

	return &m_PointCloudScene;
}

//*****************************************************************
//FUNCTION: 
void CPointCloudScene::clear()
{

}