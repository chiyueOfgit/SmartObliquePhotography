#include "pch.h"
#include "TextureSynthesizerWindow.h"
#include <QtWidgets/qmdisubwindow.h>
#include <QtWidgets/QFileDialog>
#include <qobject.h>
#include <vtkRenderWindow.h>
#include <vtkAutoInit.h>

#include <fstream>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

CTextureSynthesizerWindow::CTextureSynthesizerWindow(QWidget * vParent)
    : QMainWindow(vParent)
{
    m_WindowUI.setupUi(this);
}

CTextureSynthesizerWindow::~CTextureSynthesizerWindow()
{
}

void CTextureSynthesizerWindow::__connectSignals()
{
    QObject::connect(m_WindowUI.actionLoadInput, SIGNAL(triggered()), this, SLOT(__onActionLoadInput()));
    QObject::connect(m_WindowUI.actionLoadOutput, SIGNAL(triggered()), this, SLOT(__onActionLoadOutput()));
    QObject::connect(m_WindowUI.actionLoadMask, SIGNAL(triggered()), this, SLOT(__onActionLoadMask()));
}

std::string CTextureSynthesizerWindow::__getFileName(const std::string& vFilePath)
{
    return vFilePath.substr(vFilePath.find_last_of('/') + 1, vFilePath.find_last_of('.') - vFilePath.find_last_of('/') - 1);
}

std::string CTextureSynthesizerWindow::__getDirectory(const std::string& vFilePath)
{
    return vFilePath.substr(0, vFilePath.find_last_of('/'));
}

void CTextureSynthesizerWindow::__onActionLoadInput()
{
}

void CTextureSynthesizerWindow::__onActionLoadOutput()
{
}

void CTextureSynthesizerWindow::__onActionLoadMask()
{
}

void CTextureSynthesizerWindow::__onActionExecute()
{
}
