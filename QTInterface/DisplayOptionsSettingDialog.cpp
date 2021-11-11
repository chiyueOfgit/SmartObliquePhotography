#include "pch.h"
#include "DisplayOptionsSettingDialog.h"
#include <common/HiveConfig.h>

using namespace hiveObliquePhotography::QTInterface;
using namespace hiveObliquePhotography::SceneReconstruction;

bool CDisplayOptionsSettingDialog::m_ColorStatus = false;
bool CDisplayOptionsSettingDialog::m_NormalStatus = true;
CSceneReconstructionConfig* CDisplayOptionsSettingDialog::m_pSceneReconstructionConfig = nullptr;
int CDisplayOptionsSettingDialog::m_OctreeDepth = 0;
std::tuple<int, int> CDisplayOptionsSettingDialog::m_Resolution = std::tuple<int, int>(0, 0);
double CDisplayOptionsSettingDialog::m_SurfelRadius = 0.0;
int CDisplayOptionsSettingDialog::m_NumSample = 0;
double CDisplayOptionsSettingDialog::m_SearchRadius = 0.0;
double CDisplayOptionsSettingDialog::m_DistanceThreshold = 0.0;
double CDisplayOptionsSettingDialog::m_WeightCoefficient = 0.0;

void CDisplayOptionsSettingDialog::onActionColorFeatureCheckBox()
{
	if (m_pUi->ColorFeatureCheckBox->isChecked())
	{
		m_pUi->NormalFeatureCheckBox->setChecked(false);
	}
	m_ColorStatus = m_pUi->ColorFeatureCheckBox->isChecked();
}

void CDisplayOptionsSettingDialog::onActionNormalFeatureCheckBox()
{
	if (m_pUi->NormalFeatureCheckBox->isChecked())
	{
		m_pUi->ColorFeatureCheckBox->setChecked(false);
	}
	m_NormalStatus = m_pUi->NormalFeatureCheckBox->isChecked();
}

void hiveObliquePhotography::QTInterface::CDisplayOptionsSettingDialog::onActionOctreeDepth()
{
	m_OctreeDepth = m_pUi->OctreeDepth->value();
}

bool CDisplayOptionsSettingDialog::__initSceneReconstructionConfig()
{
	if (SceneReconstruction::hiveGetSceneReconstructionConfig(m_pSceneReconstructionConfig) == false)
		return false;

	hiveConfig::CHiveConfig* pReconstructionConfig = m_pSceneReconstructionConfig->findSubconfigByName(KEYWORD::POISSONRECONSTRUCTION);
	if (pReconstructionConfig == NULL)
		return false;

	m_OctreeDepth = pReconstructionConfig->getAttribute<int>(KEYWORD::OCTREE_DEPTH).value();

	hiveConfig::CHiveConfig* pRayCastingConfig = m_pSceneReconstructionConfig->findSubconfigByName(KEYWORD::RAYCASTING);
	if (pRayCastingConfig == NULL) 
		return false;

	m_Resolution = pRayCastingConfig->getAttribute<std::tuple<int, int>>(KEYWORD::RESOLUTION).value();
	m_SurfelRadius = pRayCastingConfig->getAttribute<double>(KEYWORD::SURFEL_RADIUS).value();
	m_NumSample = pRayCastingConfig->getAttribute<int>(KEYWORD::NUM_SAMPLE).value();
    m_SearchRadius = pRayCastingConfig->getAttribute<double>(KEYWORD::SEARCH_RADIUS).value();
	m_DistanceThreshold = pRayCastingConfig->getAttribute<double>(KEYWORD::DISTANCE_THRESHOLD).value();
	m_WeightCoefficient = pRayCastingConfig->getAttribute<double>(KEYWORD::WEIGHT_COEFFICIENT).value();

	__refreshConfigPannel();

	return true;
}

void CDisplayOptionsSettingDialog::__refreshConfigPannel()
{
	m_pUi->OctreeDepth->setValue(m_OctreeDepth);
	m_pUi->ResolutionX->setValue(std::get<0>(m_Resolution));
	m_pUi->ResolutionY->setValue(std::get<1>(m_Resolution));
	m_pUi->SurfelRadius->setValue(m_SurfelRadius);
	m_pUi->NumSample->setValue(m_NumSample);
	m_pUi->SearchRadius->setValue(m_SearchRadius);
	m_pUi->DistanceThreshold->setValue(m_DistanceThreshold);
	m_pUi->WeightCoefficent->setValue(m_WeightCoefficient);
}

void CDisplayOptionsSettingDialog::onActionResolutionXSpinBox()
{
	std::get<0>(m_Resolution) = m_pUi->ResolutionX->value();
}

void CDisplayOptionsSettingDialog::onActionResolutionYSpinBox()
{
	std::get<1>(m_Resolution) = m_pUi->ResolutionY->value();
}

void hiveObliquePhotography::QTInterface::CDisplayOptionsSettingDialog::onActionSurfelRadius()
{
	m_SurfelRadius = m_pUi->SurfelRadius->value();
}

void hiveObliquePhotography::QTInterface::CDisplayOptionsSettingDialog::onActionNumSample()
{
	m_NumSample = m_pUi->NumSample->value();
}

void hiveObliquePhotography::QTInterface::CDisplayOptionsSettingDialog::onActionSearchRadius()
{
	m_SearchRadius = m_pUi->SearchRadius->value();
}

void hiveObliquePhotography::QTInterface::CDisplayOptionsSettingDialog::onActionDistanceThreshold()
{
	m_DistanceThreshold = m_pUi->DistanceThreshold->value();
}

void hiveObliquePhotography::QTInterface::CDisplayOptionsSettingDialog::onActionWeightCoefficient()
{
	m_WeightCoefficient = m_pUi->WeightCoefficent->value();
}

void CDisplayOptionsSettingDialog::onActionOK()
{
	char* FileName = (char*)"";
	if (m_pUi->ColorFeatureCheckBox->isChecked())
	{
		FileName = const_cast<char*>("./Config/ColorFeature/PointCloudRetouchConfig.xml");
	}
	else if (m_pUi->NormalFeatureCheckBox->isChecked())
	{
		FileName = const_cast<char*>("./Config/NormalFeature/PointCloudRetouchConfig.xml");
	}
	
	if (FileName != "")
	{
		std::ifstream out(FileName);
		if (!out.is_open())
			return;
		out.close();

		WCHAR buf[256];
		memset(buf, 0, sizeof(buf));
		MultiByteToWideChar(CP_ACP, 0, FileName, strlen(FileName) + 1, buf, sizeof(buf) / sizeof(buf[0]));
		CopyFile(buf, L"./PointCloudRetouchConfig.xml", false);
	}
	
	hiveConfig::CHiveConfig* pReconstructionConfig = m_pSceneReconstructionConfig->findSubconfigByName(KEYWORD::POISSONRECONSTRUCTION);
	if (pReconstructionConfig)
	{
		pReconstructionConfig->overwriteAttribute(KEYWORD::OCTREE_DEPTH, m_OctreeDepth);
	}

	hiveConfig::CHiveConfig* pRayCastingConfig = m_pSceneReconstructionConfig->findSubconfigByName(KEYWORD::RAYCASTING);
	if (pRayCastingConfig)
	{
		pRayCastingConfig->overwriteAttribute(KEYWORD::RESOLUTION, m_Resolution);
		pRayCastingConfig->overwriteAttribute(KEYWORD::SURFEL_RADIUS, m_SurfelRadius);
		pRayCastingConfig->overwriteAttribute(KEYWORD::NUM_SAMPLE, m_NumSample);
		pRayCastingConfig->overwriteAttribute(KEYWORD::SEARCH_RADIUS, m_SearchRadius);
		pRayCastingConfig->overwriteAttribute(KEYWORD::DISTANCE_THRESHOLD, m_DistanceThreshold);
		pRayCastingConfig->overwriteAttribute(KEYWORD::WEIGHT_COEFFICIENT, m_WeightCoefficient);
	}

	__refreshConfigPannel();
}