#include "pch.h"
#include "ObliquePhotographyDataInterface.h"
#include "PointCloudScene.h"
#include "MeshLoader.h"

using namespace hiveObliquePhotography;

//*****************************************************************
//FUNCTION: 
pcl::PointCloud<pcl::PointSurfel>::Ptr hiveObliquePhotography::hiveInitPointCloudScene(const std::vector<std::string>& vFileNameSet)
{
	_ASSERTE(!vFileNameSet.empty());

	return CPointCloudScene::getInstance()->loadScene(vFileNameSet);
}

bool hiveObliquePhotography::hiveSavePointCloudScene(PointCloud_t& vPointCloud, std::string vFileName)
{
	if (vFileName.empty()||vPointCloud.empty())
		return false;
	
	return CPointCloudScene::getInstance()->saveScene(vPointCloud, vFileName);
}

void hiveObliquePhotography::hiveLoadMeshModel(hiveObliquePhotography::CMesh& voMesh, std::string vFileName)
{
	std::string LowerFileName = hiveUtility::hiveLocateFile(vFileName);
	transform(LowerFileName.begin(), LowerFileName.end(), LowerFileName.begin(), ::tolower);

	auto* pMeshLoader = hiveDesignPattern::hiveGetOrCreateProduct<IMeshLoader>(hiveUtility::hiveGetFileSuffix(vFileName));
	if (pMeshLoader)
	{
		CMesh Mesh;
		try
		{
			pMeshLoader->loadDataFromFile(Mesh, vFileName);
		}
		catch (...) {}
	}
}

std::vector<PointCloud_t::Ptr> hiveObliquePhotography::hiveGetTileSet()
{
	return CPointCloudScene::getInstance()->getTileSet();
}
