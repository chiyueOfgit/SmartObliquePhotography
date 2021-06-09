#include "pch.h"
#include "DisplayOptionsSettingDialog.h"

using namespace hiveQTInterface;

hivePointClouds::command::SRegionGrowingSetting CDisplayOptionsSettingDialog::m_GrowingSetting;

void CDisplayOptionsSettingDialog::onActionColorTest()
{
	if (m_pUi->ColorTest->isChecked())
	{
		m_GrowingSetting.bColorFlag = true;
	}
	else
	{
		m_GrowingSetting.bColorFlag = false;
	}
}

void CDisplayOptionsSettingDialog::onActionGroundTest()
{
	if (m_pUi->GroundTest->isChecked())
	{
		m_GrowingSetting.bGroundFlag = true;
	}
	else
	{
		m_GrowingSetting.bGroundFlag = false;
	}
}

void CDisplayOptionsSettingDialog::onActionNormalTest()
{
	if (m_pUi->NormalTest->isChecked())
	{
		m_GrowingSetting.bNormalFlag = true;
	}
	else
	{
		m_GrowingSetting.bNormalFlag = false;
	}
}

void CDisplayOptionsSettingDialog::onActionAverageMode()
{
	if (m_pUi->AverageButton->isChecked())
	{
		m_GrowingSetting.ColorMode = hivePointClouds::command::SRegionGrowingSetting::EColorMode::Mean;
	}
}

void CDisplayOptionsSettingDialog::onActionMedianMode()
{
	if (m_pUi->MedianButton->isChecked())
	{
		m_GrowingSetting.ColorMode = hivePointClouds::command::SRegionGrowingSetting::EColorMode::Median;
	}
}

void CDisplayOptionsSettingDialog::onActionChangeSize()
{
	m_GrowingSetting.SearchSize = m_SearchSizeSlider->value();

	m_pUi->SizeBox->setValue(m_GrowingSetting.SearchSize);
}

void CDisplayOptionsSettingDialog::onActionChangeThreshold()
{
	m_GrowingSetting.ColorThreshold = m_ColorThresholdSlider->value();

	m_pUi->ThresholdBox->setValue(m_GrowingSetting.ColorThreshold);
}

void CDisplayOptionsSettingDialog::onActionInputSize()
{
	m_GrowingSetting.SearchSize = m_pUi->SizeBox->value();

	m_SearchSizeSlider->setValue(m_GrowingSetting.SearchSize);
}

void CDisplayOptionsSettingDialog::onActionInputThreshold()
{
	m_GrowingSetting.ColorThreshold = m_pUi->ThresholdBox->value();

	m_ColorThresholdSlider->setValue(m_GrowingSetting.ColorThreshold);
}
