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

	clear();
	_ASSERTE(m_PointCloudScene->empty() && m_PointCloudTileMap.empty());

	std::vector<std::string> LoadedFiles;
	
	for (const auto& FileName : vFileNameSet)
	{
		if (find(LoadedFiles.begin(), LoadedFiles.end(), FileName) != LoadedFiles.end())
		{
			_HIVE_OUTPUT_WARNING(_FORMAT_STR1("[%1%] has already been loaded.", FileName));
			continue;
		}
		auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<IPointCloudLoader>(hiveUtility::hiveGetFileSuffix(FileName));
		if (pTileLoader)
		{
			pcl::PointCloud<pcl::PointSurfel>* pTile = pTileLoader->loadDataFromFile(FileName);
			if(!pTile)
				continue;;
			m_PointCloudTileMap.insert(std::make_pair(FileName, new CPointCloudTile(pTile)));
			*m_PointCloudScene += *pTile;
			LoadedFiles.emplace_back(FileName);
		}
		else
		{
			_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to load tile [%1%] due to unknown format.", FileName));
		}
	}
	return m_PointCloudScene;
}

//*****************************************************************
//FUNCTION: 
void CPointCloudScene::clear()
{
	for (auto Iter = m_PointCloudTileMap.begin(); Iter != m_PointCloudTileMap.end(); )
	{
		delete Iter->second;
		m_PointCloudTileMap.erase(Iter++);
	}

	//_SAFE_DELETE(m_PointCloudScene);
	m_PointCloudScene = new pcl::PointCloud<pcl::PointSurfel>;
}