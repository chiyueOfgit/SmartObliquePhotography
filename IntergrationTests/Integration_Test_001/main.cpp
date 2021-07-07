#include "pch.h"
#include "TestWindow.h"
#include <QtWidgets/QApplication>
#include <vtkOutputWindow.h>

int main(int argc, char* argv[])
{
    vtkOutputWindow::SetGlobalWarningDisplay(0);
    QApplication A(argc, argv);
    CTestWindow W;
    W.show();
    return A.exec();
}