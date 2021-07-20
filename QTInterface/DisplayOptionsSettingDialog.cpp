#include "pch.h"
#include "DisplayOptionsSettingDialog.h"

using namespace hiveObliquePhotography::QTInterface;

void CDisplayOptionsSettingDialog::onActionColorFeatureCheckBox()
{
	if (m_pUi->ColorFeatureCheckBox->isChecked())
	{
		m_pUi->PlanarFeatureCheckBox->setChecked(false);
	}
}

void CDisplayOptionsSettingDialog::onActionPlanarFeatureCheckBox()
{
	if (m_pUi->PlanarFeatureCheckBox->isChecked())
	{
		m_pUi->ColorFeatureCheckBox->setChecked(false);
	}
}

void CDisplayOptionsSettingDialog::onActionOK()
{
	char * FileName = "";
	if (m_pUi->ColorFeatureCheckBox->isChecked())
	{
		FileName = "./Config/ColorFeature/PointCloudRetouchConfig.xml";
	}
	else if (m_pUi->PlanarFeatureCheckBox->isChecked())
	{
		FileName = "./Config/PlanarFeature/PointCloudRetouchConfig.xml";
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