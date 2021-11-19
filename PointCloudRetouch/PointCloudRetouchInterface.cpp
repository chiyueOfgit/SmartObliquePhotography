#include "pch.h"
#include "PointCloudRetouchInterface.h"
#include "PointCloudRetouchManager.h"
#include "NormalComplexity.h"
#include "common/StringUtility.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::PointCloudRetouch::hiveInit(const std::vector<PointCloud_t::Ptr>& vTileSet, const hiveConfig::CHiveConfig* vConfig)
{
	_ASSERTE(!vTileSet.empty() && vConfig);
	if (_access(KEYWORD::TEMP_FOLDER.c_str(), 0) == -1)
		_mkdir(KEYWORD::TEMP_FOLDER.c_str());
	return CPointCloudRetouchManager::getInstance()->init(vTileSet, vConfig);
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::PointCloudRetouch::hiveDumpPointCloudtoSave(PointCloud_t::Ptr voPointCloud)
{
	pcl::Indices PointIndices;
	CPointCloudRetouchManager::getInstance()->dumpIndicesByLabel(PointIndices, EPointLabel::KEPT);
	CPointCloudRetouchManager::getInstance()->dumpIndicesByLabel(PointIndices, EPointLabel::UNDETERMINED);
	for (auto Index : PointIndices)
	{
		PointCloud_t::PointType TempPoint;
		auto Pos = CPointCloudRetouchManager::getInstance()->getScene().getPositionAt(Index);
		TempPoint.x = Pos.x();
		TempPoint.y = Pos.y();
		TempPoint.z = Pos.z();
		auto Normal = CPointCloudRetouchManager::getInstance()->getScene().getNormalAt(Index);
		TempPoint.normal_x = Normal.x();
		TempPoint.normal_y = Normal.y();
		TempPoint.normal_z = Normal.z();
		auto Color = CPointCloudRetouchManager::getInstance()->getScene().getColorAt(Index);
		TempPoint.r = Color.x();
		TempPoint.g = Color.y();
		TempPoint.b = Color.z();
		voPointCloud->push_back(TempPoint);
	}
	return true;
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::PointCloudRetouch::hivePreprocessSelected(std::vector<pcl::index_t>& vioSelected, const Eigen::Matrix4d& vPvMatrix, const std::function<double(Eigen::Vector2d)>& vSignedDistanceFunc, const Eigen::Vector3d& vViewPos)
{
	return CPointCloudRetouchManager::getInstance()->executePreprocessor(vioSelected, vPvMatrix, vSignedDistanceFunc, vViewPos);
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::PointCloudRetouch::hiveMarkBackground(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix, const std::function<double(Eigen::Vector2d)>& vHardnessFunc)
{
	return CPointCloudRetouchManager::getInstance()->executeMarker(vUserMarkedRegion, vPvMatrix, vHardnessFunc, EPointLabel::KEPT);
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::PointCloudRetouch::hiveMarkLitter(const std::vector<pcl::index_t>& vUserMarkedRegion, const Eigen::Matrix4d& vPvMatrix, const std::function<double(Eigen::Vector2d)>& vHardnessFunc)
{
	return CPointCloudRetouchManager::getInstance()->executeMarker(vUserMarkedRegion, vPvMatrix, vHardnessFunc, EPointLabel::UNWANTED);
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::PointCloudRetouch::hiveMarkIsolatedAreaAsLitter()
{
	return CPointCloudRetouchManager::getInstance()->executeOutlierDetector(EPointLabel::UNWANTED);
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::PointCloudRetouch::hiveDumpPointLabel(std::vector<std::size_t>& voPointLabel)
{
	return CPointCloudRetouchManager::getInstance()->dumpPointLabel(voPointLabel);
}

//*****************************************************************
//FUNCTION: 
bool hiveObliquePhotography::PointCloudRetouch::hiveDumpTileLabel(std::size_t vTile, std::vector<std::size_t>& voTileLabel)
{
	return CPointCloudRetouchManager::getInstance()->dumpTileLabel(vTile, voTileLabel);
}

void hiveObliquePhotography::PointCloudRetouch::hiveDumpExpandResult(std::vector<pcl::index_t>& voExpandPoints, bool vIsLitterMarker)
{
	CPointCloudRetouchManager::getInstance()->dumpExpandPoints(voExpandPoints, vIsLitterMarker);
}

void hiveObliquePhotography::PointCloudRetouch::hiveHideLitter()
{
	CPointCloudRetouchManager::getInstance()->switchLabel(EPointLabel::DISCARDED, EPointLabel::UNWANTED);
}

void hiveObliquePhotography::PointCloudRetouch::hiveDisplayLitter()
{
	CPointCloudRetouchManager::getInstance()->switchLabel(EPointLabel::UNWANTED, EPointLabel::DISCARDED);
}

void hiveObliquePhotography::PointCloudRetouch::hiveClearMark()
{
	CPointCloudRetouchManager::getInstance()->clearMark();
}

void hiveObliquePhotography::PointCloudRetouch::hiveRecoverLitterMark()
{
	CPointCloudRetouchManager::getInstance()->recoverMarkedPoints2Undetermined(EPointLabel::UNWANTED);
}

void hiveObliquePhotography::PointCloudRetouch::hiveRecoverBackgroundMark()
{
	CPointCloudRetouchManager::getInstance()->recoverMarkedPoints2Undetermined(EPointLabel::KEPT);
}

void hiveObliquePhotography::PointCloudRetouch::hiveEraseMark(const std::vector<pcl::index_t>& vPoints)
{
	CPointCloudRetouchManager::getInstance()->setLabel(vPoints, EPointLabel::UNDETERMINED);
}

bool hiveObliquePhotography::PointCloudRetouch::hiveDumpColorFeatureMainColors(std::vector<Eigen::Vector3i>& voMainColors)
{
	return CPointCloudRetouchManager::getInstance()->dumpColorFeatureMainColors(voMainColors);
}

bool hiveObliquePhotography::PointCloudRetouch::hiveDumpColorFeatureNearestPoints(std::vector<pcl::index_t>& vNearestPoints)
{
	return CPointCloudRetouchManager::getInstance()->dumpColorFeatureNearestPoints(vNearestPoints);
}

bool hiveObliquePhotography::PointCloudRetouch::hiveUndo()
{
	return CPointCloudRetouchManager::getInstance()->undo();
}

void hiveObliquePhotography::PointCloudRetouch::hiveRunPrecompute(const std::string& vModelName)
{
	auto pFeature = dynamic_cast<CNormalComplexity*>(hiveDesignPattern::hiveGetOrCreateProduct<IFeature>(KEYWORD::NORMAL_COMPLEXITY));
	auto pPrecomputeManager = CPointCloudRetouchManager::getInstance()->getPrecomputeManager();
	pFeature->initV(pPrecomputeManager->getFeatureConfig(KEYWORD::NORMAL_COMPLEXITY));

	if (_access(KEYWORD::TEMP_FOLDER.c_str(), 0) == -1)
		_mkdir(KEYWORD::TEMP_FOLDER.c_str());
	pPrecomputeManager->registerPrecompute<std::vector<double>>([=]()->bool {return pFeature->precomputeSceneCloudNormalComplexity(); }, KEYWORD::TEMP_FOLDER + vModelName + "_" + std::to_string(CPointCloudRetouchManager::getInstance()->getScene().getNumPoint()) + "_pre.txt", pFeature->getPtr2Container());

	pPrecomputeManager->precompute();
}

void hiveObliquePhotography::PointCloudRetouch::hiveRepairHoleSetRepairRegion(std::vector<pcl::index_t>& vRepairRegion)
{
	CPointCloudRetouchManager::getInstance()->executeHoleRepairerSetRegion(vRepairRegion);
}

void hiveObliquePhotography::PointCloudRetouch::hiveRepairHoleSetReferenceRegion(std::vector<pcl::index_t>& vReferenceRegion)
{
	CPointCloudRetouchManager::getInstance()->executeHoleRepairerSetInput(vReferenceRegion);
}

void hiveObliquePhotography::PointCloudRetouch::hiveRepairHole(std::vector<pcl::PointXYZRGBNormal>& voNewPoints)
{
	CPointCloudRetouchManager::getInstance()->executeHoleRepairer(voNewPoints);
}

void hiveObliquePhotography::PointCloudRetouch::hiveTagLabel(const std::vector<pcl::index_t>& vPoints, bool vIsLitterMarker)	//only for perform
{
	EPointLabel AimLabel;
	if (vIsLitterMarker)
		AimLabel = EPointLabel::UNWANTED;
	else
		AimLabel = EPointLabel::KEPT;
	for (auto Index : vPoints)
		CPointCloudRetouchManager::getInstance()->tagPointLabel(Index, AimLabel, 0, 0);
}

void hiveObliquePhotography::PointCloudRetouch::hiveAutoMarkLitter()
{
	CPointCloudRetouchManager::getInstance()->executeAutoMarker();
}

 void hiveObliquePhotography::PointCloudRetouch::hiveAutoHolerepair(std::vector<pcl::PointXYZRGBNormal>& voNewPointSet)
{
	 CPointCloudRetouchManager::getInstance()->executeAutoHoleRepair(voNewPointSet);
}