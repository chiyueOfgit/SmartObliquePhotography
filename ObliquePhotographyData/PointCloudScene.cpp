#include "pch.h"
#include "PointCloudScene.h"
#include <boost/format.hpp>
#include "common/UtilityInterface.h"
#include "PointCloudLoader.h"

using namespace hiveObliquePhotography;

CPointCloudScene::~CPointCloudScene()
{
	clear();
}

//*****************************************************************
//FUNCTION: 
PointCloud_t::Ptr CPointCloudScene::loadScene(const std::vector<std::string>& vFileNameSet)
{
	_ASSERTE(!vFileNameSet.empty());

	clear();
	m_pPointCloudScene.reset(new PointCloud_t);
	
	_ASSERTE(m_pPointCloudScene->empty() && m_PointCloudTileMap.empty());

	std::vector<std::string> LoadedFileSet;
	
	for (const auto& FileName : vFileNameSet)
	{
		if (find(LoadedFileSet.begin(), LoadedFileSet.end(), FileName) != LoadedFileSet.end())
		{
			_HIVE_OUTPUT_WARNING(_FORMAT_STR1("[%1%] has already been loaded.", FileName));
			continue;
		}
		auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<IPointCloudLoader>(hiveUtility::hiveGetFileSuffix(FileName));
		if (pTileLoader)
		{
			auto pTile = pTileLoader->loadDataFromFile(FileName);
			if(!pTile) continue;
			m_PointCloudTileMap.emplace(FileName, pTile);
			*m_pPointCloudScene += *pTile;
			LoadedFileSet.emplace_back(FileName);
		}
		else
		{
			_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to load tile [%1%] due to unknown format.", FileName));
		}
	}
	return m_pPointCloudScene;
}

//*****************************************************************
//FUNCTION: 
void CPointCloudScene::clear()
{
	m_PointCloudTileMap.clear();
	m_pPointCloudScene.reset();
}