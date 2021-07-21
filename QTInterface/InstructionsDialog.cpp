#include "pch.h"
#include "InstructionsDialog.h"
#include <fstream>
#include <iostream>

using namespace hiveObliquePhotography::QTInterface;

void CInstructionsDialog::init()
{
	__loadTxt();
	__setLabelText();
	__connectSignals();
}

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
			m_LabelText.push_back(Line);
		}
	}
}

void CInstructionsDialog::__setLabelText()
{
	for (auto Line : m_LabelText)
	{
		m_pUi->RichText->append(QString(Line.c_str()));
	}
}

void CInstructionsDialog::onButtonOKClicked()
{
	this->close();
}
