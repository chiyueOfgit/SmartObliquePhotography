#include "pch.h"
#include "PointCloudScene.h"

#include <common/FileSystem.h>
#include <string>
#include "PointCloudLoader.h"
#include "PointCloudSaver.h"
#include <filesystem>

using namespace hiveObliquePhotography;

CPointCloudScene::~CPointCloudScene()
{
	clear();
}

//*****************************************************************
//FUNCTION: 
std::vector<PointCloud_t::Ptr> CPointCloudScene::loadScene(const std::vector<std::string>& vFileNameSet)
{
	if (vFileNameSet.empty()) return {};
	
	clear();
	_ASSERTE(m_PointCloudTileMap.empty());

	std::vector<std::string> LoadedFileSet;
	
	for (const auto& FileName : vFileNameSet)
	{
		std::string LowerFileName = hiveUtility::hiveLocateFile(FileName);
		if(LowerFileName.empty())
		{
			_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to load file [%1%] because it does not exist.", FileName));
			continue;
		}
		transform(LowerFileName.begin(), LowerFileName.end(), LowerFileName.begin(), ::tolower);

		if (find(LoadedFileSet.begin(), LoadedFileSet.end(), LowerFileName) != LoadedFileSet.end())
		{
			_HIVE_OUTPUT_WARNING(_FORMAT_STR1("[%1%] has already been loaded.", FileName));
			continue;
		}
		
		auto* pTileLoader = hiveDesignPattern::hiveGetOrCreateProduct<IPointCloudLoader>(hiveUtility::hiveGetFileSuffix(FileName));
		if (pTileLoader)
		{
			std::shared_ptr<PointCloud_t> pTile = nullptr;
			try
			{
				pTile = pTileLoader->loadDataFromFile(FileName);
			}
			catch (...) { }

			if(!pTile) continue;
			m_TileSet.push_back(pTile);
			m_PointCloudTileMap.emplace(FileName, pTile);
			LoadedFileSet.emplace_back(LowerFileName);
		}
		else
		{
			_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to load tile [%1%] due to unknown format.", FileName));
		}
	}
	return m_TileSet;
}

//*****************************************************************
//FUNCTION: 
bool CPointCloudScene::saveScene(PointCloud_t& vPointCloud, std::string vFileName)
{
	auto* pSceneSaver = hiveDesignPattern::hiveGetOrCreateProduct<IPointCloudSaver>(hiveUtility::hiveGetFileSuffix(vFileName) + "_Save");
	if (pSceneSaver)
	{
		pSceneSaver->saveDataToFileV(vPointCloud, vFileName);
	}
	else
	{
		_HIVE_OUTPUT_WARNING(_FORMAT_STR1("Fail to save tile [%1%] due to unknown format.", vFileName));
	}
	return true;
}

//*****************************************************************
//FUNCTION: 
void CPointCloudScene::clear()
{
	m_PointCloudTileMap.clear();
}