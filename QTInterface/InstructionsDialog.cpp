#include "pch.h"
#include "InstructionsDialog.h"
#include <fstream>
#include <iostream>

using namespace hiveObliquePhotography::QTInterface;

void CInstructionsDialog::__connectSignals()
{
	QObject::connect(m_pUi->ButtonOK, SIGNAL(clicked()), this, SLOT(onButtonOKClicked()));
}

void CInstructionsDialog::__loadTxt()
{
	std::ifstream File("Instructions.txt");
	std::vector<std::string> TextString;
	std::string Line = "";

	if (!File.is_open())
		return;

	while (std::getline(File, Line))
	{
		if (Line.size() > 0)
		{
			m_LabelText += Line;
			m_LabelText += "\n";
		}
	}
}

void CInstructionsDialog::__setLabelText()
{
	m_pUi->labelText->setText(QString(m_LabelText.c_str()));
}

void CInstructionsDialog::onButtonOKClicked()
{
	this->close();
}
