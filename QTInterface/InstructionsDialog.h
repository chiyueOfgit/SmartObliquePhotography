#pragma once
#include "ui_InstructionsDialog.h"
#include <qdialog.h>
#include "FloatSlider.h"
#include <fstream>
#include <iostream>

namespace hiveObliquePhotography
{
	namespace QTInterface
	{
		class CInstructionsDialog : public QDialog
		{
			Q_OBJECT

		public:
			CInstructionsDialog(QWidget* vParent)
				: QDialog(vParent),
				m_pUi(std::make_shared<Ui::CInstructionsDialog>())
			{
				m_pUi->setupUi(this);

				__loadTxt();
				__setLabelText();
				__connectSignals();
			}

		private:
			std::shared_ptr<Ui::CInstructionsDialog> m_pUi = nullptr;
			std::string m_LabelText = "";

			void __loadTxt();
			void __connectSignals();
			void __setLabelText();

		public slots:
			void onButtonOKClicked();
		};
	}
}