#pragma once

#include "ui_DisplayOptionsSettingDialog.h"
#include <qdialog.h>

namespace hiveObliquePhotography::QTInterface
{
	class CDisplayOptionsSettingDialog : public QDialog
	{
		Q_OBJECT

	public:
		CDisplayOptionsSettingDialog(QWidget* vParent, CDisplayOptionsSettingDialog*& vThisInParent)
			: QDialog(vParent), m_ThisInParent(vThisInParent),
			m_pUi(new Ui::CDisplayOptionsSettingDialog)
		{
			this->setAttribute(Qt::WA_DeleteOnClose);

			m_pUi->setupUi(this);
			this->setWindowFlag(Qt::WindowType::WindowContextHelpButtonHint, false);

			m_pUi->ColorFeatureCheckBox->setChecked(m_ColorStatus);
			m_pUi->NormalFeatureCheckBox->setChecked(m_NormalStatus);

			char* FileName = "./Config/NormalFeature/PointCloudRetouchConfig.xml";
			std::ifstream out(FileName);
			if (!out.is_open())
				return;
			out.close();

			WCHAR buf[256];
			memset(buf, 0, sizeof(buf));
			MultiByteToWideChar(CP_ACP, 0, FileName, strlen(FileName) + 1, buf, sizeof(buf) / sizeof(buf[0]));
			CopyFile(buf, L"./PointCloudRetouchConfig.xml", false);

			QObject::connect(m_pUi->ColorFeatureCheckBox, SIGNAL(stateChanged(int)), this, SLOT(onActionColorFeatureCheckBox()));
			QObject::connect(m_pUi->NormalFeatureCheckBox, SIGNAL(stateChanged(int)), this, SLOT(onActionNormalFeatureCheckBox()));
			QObject::connect(m_pUi->OKButton, SIGNAL(clicked()), this, SLOT(onActionOK()));
		}
		
		~CDisplayOptionsSettingDialog()
		{
			//通知主窗口该窗口已经关闭
			m_ThisInParent = nullptr;
		}

	private:
		static bool m_ColorStatus;
		static bool m_NormalStatus;

	private slots:
		void onActionColorFeatureCheckBox();
		void onActionNormalFeatureCheckBox();
		void onActionOK();

	private:
		CDisplayOptionsSettingDialog*& m_ThisInParent;
		Ui::CDisplayOptionsSettingDialog* m_pUi = nullptr;

	};
}