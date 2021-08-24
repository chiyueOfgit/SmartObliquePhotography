#include "pch.h"
#include "TextureSynthesizerWindow.h"
#include <QtWidgets/QApplication>
#include <vtkOutputWindow.h>

int main(int argc, char* argv[])
{
    vtkOutputWindow::SetGlobalWarningDisplay(0);
    QApplication A(argc, argv);
    CTextureSynthesizerWindow W;
    W.show();
    return A.exec();
}