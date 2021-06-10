#include "pch.h"
#include "DisplayOptionsSettingDialog.h"

using namespace hiveQTInterface;

void CDisplayOptionsSettingDialog::onActionColorTest()
{
	if (m_pUi->ColorTest->isChecked())
	{
		CAutoRetouchConfig::getInstance()->overwriteAttribute("ENABLE_COLOR_TEST", true);
	}
	else
	{
		CAutoRetouchConfig::getInstance()->overwriteAttribute("ENABLE_COLOR_TEST", false);
	}
}

void CDisplayOptionsSettingDialog::onActionGroundTest()
{
	if (m_pUi->GroundTest->isChecked())
	{
		CAutoRetouchConfig::getInstance()->overwriteAttribute("ENABLE_GROUND_TEST", true);
	}
	else
	{
		CAutoRetouchConfig::getInstance()->overwriteAttribute("ENABLE_GROUND_TEST", false);
	}
}

void CDisplayOptionsSettingDialog::onActionNormalTest()
{
	if (m_pUi->NormalTest->isChecked())
	{
		CAutoRetouchConfig::getInstance()->overwriteAttribute("ENABLE_NORMAL_TEST", true);
	}
	else
	{
		CAutoRetouchConfig::getInstance()->overwriteAttribute("ENABLE_NORMAL_TEST", false);
	}
}

void CDisplayOptionsSettingDialog::onActionAverageMode()
{
	if (m_pUi->AverageButton->isChecked())
	{
		CAutoRetouchConfig::getInstance()->overwriteAttribute("COLOR_TEST_MODE", 0);
	}
}

void CDisplayOptionsSettingDialog::onActionMedianMode()
{
	if (m_pUi->MedianButton->isChecked())
	{
		CAutoRetouchConfig::getInstance()->overwriteAttribute("COLOR_TEST_MODE", 1);
	}
}

void CDisplayOptionsSettingDialog::onActionChangeSize()
{
	CAutoRetouchConfig::getInstance()->overwriteAttribute("SEARCH_RADIUS", m_SearchSizeSlider->value());

	m_pUi->SizeBox->setValue(*CAutoRetouchConfig::getInstance()->getAttribute<float>("SEARCH_RADIUS"));
}

void CDisplayOptionsSettingDialog::onActionChangeThreshold()
{
	CAutoRetouchConfig::getInstance()->overwriteAttribute("COLOR_TEST_THRESHOLD", m_ColorThresholdSlider->value());
	
	m_pUi->ThresholdBox->setValue(*CAutoRetouchConfig::getInstance()->getAttribute<float>("COLOR_TEST_THRESHOLD"));
	
}

void CDisplayOptionsSettingDialog::onActionInputSize()
{
	CAutoRetouchConfig::getInstance()->overwriteAttribute("SEARCH_RADIUS", m_pUi->SizeBox->value());
	m_SearchSizeSlider->setValue(*CAutoRetouchConfig::getInstance()->getAttribute<float>("SEARCH_RADIUS"));
}

void CDisplayOptionsSettingDialog::onActionInputThreshold()
{
	CAutoRetouchConfig::getInstance()->overwriteAttribute("COLOR_TEST_THRESHOLD", m_ColorThresholdSlider->value());

	m_ColorThresholdSlider->setValue(*CAutoRetouchConfig::getInstance()->getAttribute<float>("COLOR_TEST_THRESHOLD"));
}
