#include "pch.h"
#include "DisplayOptionsSettingDialog.h"

using namespace hiveQTInterface;

hivePointClouds::command::SRegionGrowingSetting CDisplayOptionsSettingDialog::m_GrowingSetting;

void CDisplayOptionsSettingDialog::onActionColorTest()
{
	if (m_pUi->ColorTest->isChecked())
	{
		CAutoRetouchConfig::getInstance()->overwriteAttribute("ENABLE_COLOR_TEST", true);
		//m_GrowingSetting.bColorFlag = true;
	}
	else
	{
		//m_GrowingSetting.bColorFlag = false;
		CAutoRetouchConfig::getInstance()->overwriteAttribute("ENABLE_COLOR_TEST", false);
	}
}

void CDisplayOptionsSettingDialog::onActionGroundTest()
{
	if (m_pUi->GroundTest->isChecked())
	{
		//m_GrowingSetting.bGroundFlag = true;
		CAutoRetouchConfig::getInstance()->overwriteAttribute("ENABLE_GROUND_TEST", true);
	}
	else
	{
		//m_GrowingSetting.bGroundFlag = false;
		CAutoRetouchConfig::getInstance()->overwriteAttribute("ENABLE_GROUND_TEST", false);
	}
}

void CDisplayOptionsSettingDialog::onActionNormalTest()
{
	if (m_pUi->NormalTest->isChecked())
	{
		//m_GrowingSetting.bNormalFlag = true;
		CAutoRetouchConfig::getInstance()->overwriteAttribute("ENABLE_NORMAL_TEST", true);
	}
	else
	{
		//m_GrowingSetting.bNormalFlag = false;
		CAutoRetouchConfig::getInstance()->overwriteAttribute("ENABLE_NORMAL_TEST", false);
	}
}

void CDisplayOptionsSettingDialog::onActionAverageMode()
{
	if (m_pUi->AverageButton->isChecked())
	{
		//m_GrowingSetting.ColorMode = hivePointClouds::command::SRegionGrowingSetting::EColorMode::Mean;
		CAutoRetouchConfig::getInstance()->overwriteAttribute("COLOR_TEST_MODE", 0);
	}
}

void CDisplayOptionsSettingDialog::onActionMedianMode()
{
	if (m_pUi->MedianButton->isChecked())
	{
		//m_GrowingSetting.ColorMode = hivePointClouds::command::SRegionGrowingSetting::EColorMode::Median;
		CAutoRetouchConfig::getInstance()->overwriteAttribute("COLOR_TEST_MODE", 1);
	}
}

void CDisplayOptionsSettingDialog::onActionChangeSize()
{
	//m_GrowingSetting.SearchSize = m_SearchSizeSlider->value();
	CAutoRetouchConfig::getInstance()->overwriteAttribute("SEARCH_RADIUS", m_SearchSizeSlider->value());

	m_pUi->SizeBox->setValue(*CAutoRetouchConfig::getInstance()->getAttribute<float>("SEARCH_RADIUS"));
}

void CDisplayOptionsSettingDialog::onActionChangeThreshold()
{
	//m_GrowingSetting.ColorThreshold = m_ColorThresholdSlider->value();
	CAutoRetouchConfig::getInstance()->overwriteAttribute("COLOR_TEST_THRESHOLD", m_ColorThresholdSlider->value());
	
	m_pUi->ThresholdBox->setValue(*CAutoRetouchConfig::getInstance()->getAttribute<float>("COLOR_TEST_THRESHOLD"));
	
}

void CDisplayOptionsSettingDialog::onActionInputSize()
{
	//m_GrowingSetting.SearchSize = m_pUi->SizeBox->value();
	CAutoRetouchConfig::getInstance()->overwriteAttribute("SEARCH_RADIUS", m_pUi->SizeBox->value());
	m_SearchSizeSlider->setValue(*CAutoRetouchConfig::getInstance()->getAttribute<float>("SEARCH_RADIUS"));
}

void CDisplayOptionsSettingDialog::onActionInputThreshold()
{
	//m_GrowingSetting.ColorThreshold = m_pUi->ThresholdBox->value();
	CAutoRetouchConfig::getInstance()->overwriteAttribute("COLOR_TEST_THRESHOLD", m_ColorThresholdSlider->value());

	m_ColorThresholdSlider->setValue(*CAutoRetouchConfig::getInstance()->getAttribute<float>("COLOR_TEST_THRESHOLD"));
}
