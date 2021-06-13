#include "pch.h"
#include "DisplayOptionsSettingDialog.h"

using namespace hiveQTInterface;

void CDisplayOptionsSettingDialog::onActionColorTest()
{
	if (m_pUi->ColorTest->isChecked())
	{
		hiveObliquePhotography::AutoRetouch::CAutoRetouchConfig::getInstance()->overwriteAttribute(KEY_WORDS::ENABLE_COLOR_TEST, true);
	}
	else
	{
		hiveObliquePhotography::AutoRetouch::CAutoRetouchConfig::getInstance()->overwriteAttribute(KEY_WORDS::ENABLE_COLOR_TEST, false);
	}
}

void CDisplayOptionsSettingDialog::onActionGroundTest()
{
	if (m_pUi->GroundTest->isChecked())
	{
		hiveObliquePhotography::AutoRetouch::CAutoRetouchConfig::getInstance()->overwriteAttribute(KEY_WORDS::ENABLE_GROUND_TEST, true);
	}
	else
	{
		hiveObliquePhotography::AutoRetouch::CAutoRetouchConfig::getInstance()->overwriteAttribute(KEY_WORDS::ENABLE_GROUND_TEST, false);
	}
}

void CDisplayOptionsSettingDialog::onActionNormalTest()
{
	if (m_pUi->NormalTest->isChecked())
	{
		hiveObliquePhotography::AutoRetouch::CAutoRetouchConfig::getInstance()->overwriteAttribute(KEY_WORDS::ENABLE_NORMAL_TEST, true);
	}
	else
	{
		hiveObliquePhotography::AutoRetouch::CAutoRetouchConfig::getInstance()->overwriteAttribute(KEY_WORDS::ENABLE_NORMAL_TEST, false);
	}
}

void CDisplayOptionsSettingDialog::onActionAverageMode()
{
	if (m_pUi->AverageButton->isChecked())
	{
		hiveObliquePhotography::AutoRetouch::CAutoRetouchConfig::getInstance()->overwriteAttribute(KEY_WORDS::COLOR_TEST_MODE, 0);
	}
}

void CDisplayOptionsSettingDialog::onActionMedianMode()
{
	if (m_pUi->MedianButton->isChecked())
	{
		hiveObliquePhotography::AutoRetouch::CAutoRetouchConfig::getInstance()->overwriteAttribute(KEY_WORDS::COLOR_TEST_MODE, 1);
	}
}

void CDisplayOptionsSettingDialog::onActionChangeSize()
{
	hiveObliquePhotography::AutoRetouch::CAutoRetouchConfig::getInstance()->overwriteAttribute(KEY_WORDS::SEARCH_RADIUS, m_SearchSizeSlider->value());

	m_pUi->SizeBox->setValue(*hiveObliquePhotography::AutoRetouch::CAutoRetouchConfig::getInstance()->getAttribute<double>(KEY_WORDS::SEARCH_RADIUS));
}

void CDisplayOptionsSettingDialog::onActionChangeThreshold()
{
	hiveObliquePhotography::AutoRetouch::CAutoRetouchConfig::getInstance()->overwriteAttribute(KEY_WORDS::COLOR_TEST_THRESHOLD, m_ColorThresholdSlider->value());
	
	m_pUi->ThresholdBox->setValue(*hiveObliquePhotography::AutoRetouch::CAutoRetouchConfig::getInstance()->getAttribute<float>(KEY_WORDS::COLOR_TEST_THRESHOLD));
	
}

void CDisplayOptionsSettingDialog::onActionInputSize()
{
	hiveObliquePhotography::AutoRetouch::CAutoRetouchConfig::getInstance()->overwriteAttribute(KEY_WORDS::SEARCH_RADIUS, m_pUi->SizeBox->value());
	m_SearchSizeSlider->setValue(static_cast<float>(*hiveObliquePhotography::AutoRetouch::CAutoRetouchConfig::getInstance()->getAttribute<double>(KEY_WORDS::SEARCH_RADIUS)));
}

void CDisplayOptionsSettingDialog::onActionInputThreshold()
{
	hiveObliquePhotography::AutoRetouch::CAutoRetouchConfig::getInstance()->overwriteAttribute(KEY_WORDS::COLOR_TEST_THRESHOLD, m_ColorThresholdSlider->value());

	m_ColorThresholdSlider->setValue(*hiveObliquePhotography::AutoRetouch::CAutoRetouchConfig::getInstance()->getAttribute<float>(KEY_WORDS::COLOR_TEST_THRESHOLD));
}
