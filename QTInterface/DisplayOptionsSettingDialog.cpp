#include "pch.h"
#include "DisplayOptionsSettingDialog.h"

using namespace hiveObliquePhotography::QTInterface;

bool CDisplayOptionsSettingDialog::m_ColorStatus = false;
bool CDisplayOptionsSettingDialog::m_NormalStatus = true;

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
	
}