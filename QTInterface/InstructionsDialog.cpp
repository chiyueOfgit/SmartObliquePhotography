#include "pch.h"
#include "InstructionsDialog.h"
#include "QTInterfaceConfig.h"
#include <qscrollbar.h>
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
	int LineNumber = 0;

	if (!File.is_open())
		return;

	while (std::getline(File, Line))
	{
		LineNumber++;
		if (Line.size() > 0)
		{
			if (Line[0] == '#' && LineNumber > 1)
				m_LabelText.push_back("\n");
			if (Line.size() == 2 && Line[0] == '\\' && Line[1] == 'n')
			{
				m_LabelText.push_back("\n");
				continue;
			}
			m_LabelText.push_back(Line);
		}
	}
}

void CInstructionsDialog::__setLabelText()
{
	auto RichTextFont = CQInterfaceConfig::getInstance()->getAttribute<std::tuple<int, int>>("INSTRUCTIONS_FONT_STYLE").value();
	m_pUi->RichText->setFont(QFont("Microsoft YaHe", std::get<0>(RichTextFont), std::get<1>(RichTextFont)));
	for (auto Line : m_LabelText)
	{
		m_pUi->RichText->append(QString(Line.c_str()));
	}
	m_pUi->RichText->moveCursor(QTextCursor::Start);
}

void CInstructionsDialog::onButtonOKClicked()
{
	this->close();
}
