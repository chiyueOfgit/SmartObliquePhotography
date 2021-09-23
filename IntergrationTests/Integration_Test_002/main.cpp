#include "pch.h"
#include "SingleStepWindow.h"
#include <QtWidgets/QApplication>
#include <vtkOutputWindow.h>

int main(int argc, char* argv[])
{
    vtkOutputWindow::SetGlobalWarningDisplay(0);
    QApplication A(argc, argv);
    CSingleStepWindow W;
    W.show();
    return A.exec();
}