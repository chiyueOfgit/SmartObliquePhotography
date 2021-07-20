#pragma once

#include "ui_DisplayOptionsSettingDialog.h"
#include <qdialog.h>

namespace hiveObliquePhotography::QTInterface
{
	class CDisplayOptionsSettingDialog : public QDialog
	{
		Q_OBJECT

	public:
		CDisplayOptionsSettingDialog(QWidget* vParent)
			: QDialog(vParent),
			m_pUi(std::make_shared<Ui::CDisplayOptionsSettingDialog>())
		{
			m_pUi->setupUi(this);

			this->setWindowFlag(Qt::WindowType::WindowContextHelpButtonHint, false);

			m_pUi->ColorFeatureCheckBox->setChecked(m_ColorStatus);
			m_pUi->PlanarFeatureCheckBox->setChecked(m_PlanarStatus);

			QObject::connect(m_pUi->ColorFeatureCheckBox, SIGNAL(stateChanged(int)), this, SLOT(onActionColorFeatureCheckBox()));
			QObject::connect(m_pUi->PlanarFeatureCheckBox, SIGNAL(stateChanged(int)), this, SLOT(onActionPlanarFeatureCheckBox()));
			QObject::connect(m_pUi->OKButton, SIGNAL(clicked()), this, SLOT(onActionOK()));
		}
		
	private:
		static bool m_ColorStatus;
		static bool m_PlanarStatus;

	private slots:
		void onActionColorFeatureCheckBox();
		void onActionPlanarFeatureCheckBox();
		void onActionOK();

	private:
		std::shared_ptr<Ui::CDisplayOptionsSettingDialog> m_pUi = nullptr;

	};
}