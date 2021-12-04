#include "pch.h"
#include "ObliquePhotographyDataInterface.h"
#include "PointCloudScene.h"
#include "MeshLoader.h"
#include "MeshSaver.h"

using namespace hiveObliquePhotography;

//*****************************************************************
//FUNCTION: 
std::vector<PointCloud_t::Ptr> hiveObliquePhotography::hiveInitPointCloudScene(const std::vector<std::string>& vFileNameSet)
{
	if (vFileNameSet.empty()) return {};

	return CPointCloudScene::getInstance()->loadScene(vFileNameSet);
}

bool hiveObliquePhotography::hiveSavePointCloudScene(PointCloud_t::Ptr vPointCloud, const std::string& vFileName)
{
	if (vFileName.empty()||vPointCloud->empty())
		return false;
	
	return CPointCloudScene::getInstance()->saveScene(vPointCloud, vFileName);
}

void hiveObliquePhotography::hiveLoadMeshModel(hiveObliquePhotography::CMesh& voMesh, const std::string& vFileName)
{
	std::string LowerFileName = hiveUtility::hiveLocateFile(vFileName);
	transform(LowerFileName.begin(), LowerFileName.end(), LowerFileName.begin(), ::tolower);

	auto* pMeshLoader = hiveDesignPattern::hiveGetOrCreateProduct<IMeshLoader>(hiveUtility::hiveGetFileSuffix(vFileName));
	if (pMeshLoader)
	{
		try
		{
			pMeshLoader->loadDataFromFile(voMesh, vFileName);
		}
		catch (...) {}
	}
}

void hiveObliquePhotography::hiveSaveMeshModel(const hiveObliquePhotography::CMesh& vMesh, const std::string& vFileName)
{
	std::string LowerFileName = hiveUtility::hiveLocateFile(vFileName);
	transform(LowerFileName.begin(), LowerFileName.end(), LowerFileName.begin(), ::tolower);

	auto* pMeshLoader = hiveDesignPattern::hiveGetOrCreateProduct<IMeshSaver>(hiveUtility::hiveGetFileSuffix(vFileName) + "_Save");
	if (pMeshLoader)
	{
		try
		{
			pMeshLoader->saveDataToFileV(vMesh, vFileName);
		}
		catch (...) {}
	}
}
