#include "pch.h"
#include "DisplayOptionsSettingDialog.h"

using namespace hiveObliquePhotography::QTInterface;

void CDisplayOptionsSettingDialog::onActionColorTest()
{
	if (m_pUi->ColorTest->isChecked())
	{
		m_pAutoRetouchConfig->overwriteAttribute(KEY_WORDS::ENABLE_COLOR_TEST, true);
	}
	else
	{
		m_pAutoRetouchConfig->overwriteAttribute(KEY_WORDS::ENABLE_COLOR_TEST, false);
	}
}

void CDisplayOptionsSettingDialog::onActionGroundTest()
{
	if (m_pUi->GroundTest->isChecked())
	{
		m_pAutoRetouchConfig->overwriteAttribute(KEY_WORDS::ENABLE_GROUND_TEST, true);
	}
	else
	{
		m_pAutoRetouchConfig->overwriteAttribute(KEY_WORDS::ENABLE_GROUND_TEST, false);
	}
}

void CDisplayOptionsSettingDialog::onActionNormalTest()
{
	if (m_pUi->NormalTest->isChecked())
	{
		m_pAutoRetouchConfig->overwriteAttribute(KEY_WORDS::ENABLE_NORMAL_TEST, true);
	}
	else
	{
		m_pAutoRetouchConfig->overwriteAttribute(KEY_WORDS::ENABLE_NORMAL_TEST, false);
	}
}

void CDisplayOptionsSettingDialog::onActionAverageMode()
{
	if (m_pUi->AverageButton->isChecked())
	{
		m_pAutoRetouchConfig->overwriteAttribute(KEY_WORDS::COLOR_TEST_MODE, 0);
	}
}

void CDisplayOptionsSettingDialog::onActionMedianMode()
{
	if (m_pUi->MedianButton->isChecked())
	{
		m_pAutoRetouchConfig->overwriteAttribute(KEY_WORDS::COLOR_TEST_MODE, 1);
	}
}

void CDisplayOptionsSettingDialog::onActionChangeSize()
{
	m_pAutoRetouchConfig->overwriteAttribute(KEY_WORDS::SEARCH_RADIUS, m_SearchSizeSlider->value());

	m_pUi->SizeBox->setValue(*m_pAutoRetouchConfig->getAttribute<double>(KEY_WORDS::SEARCH_RADIUS));
}

void CDisplayOptionsSettingDialog::onActionChangeThreshold()
{
	m_pAutoRetouchConfig->overwriteAttribute(KEY_WORDS::COLOR_TEST_THRESHOLD, m_ColorThresholdSlider->value());
	
	m_pUi->ThresholdBox->setValue(*m_pAutoRetouchConfig->getAttribute<float>(KEY_WORDS::COLOR_TEST_THRESHOLD));
	
}

void CDisplayOptionsSettingDialog::onActionInputSize()
{
	m_pAutoRetouchConfig->overwriteAttribute(KEY_WORDS::SEARCH_RADIUS, m_pUi->SizeBox->value());
	m_SearchSizeSlider->setValue(static_cast<float>(*m_pAutoRetouchConfig->getAttribute<double>(KEY_WORDS::SEARCH_RADIUS)));
}

void CDisplayOptionsSettingDialog::onActionInputThreshold()
{
	m_pAutoRetouchConfig->overwriteAttribute(KEY_WORDS::COLOR_TEST_THRESHOLD, m_ColorThresholdSlider->value());

	m_ColorThresholdSlider->setValue(*m_pAutoRetouchConfig->getAttribute<float>(KEY_WORDS::COLOR_TEST_THRESHOLD));
}
