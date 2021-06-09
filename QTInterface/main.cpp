#include "pch.h"
#include "QTInterface.h"
#include <QtWidgets/QApplication>
#include <vtkOutputWindow.h>

int main(int argc, char *argv[])
{
    vtkOutputWindow::SetGlobalWarningDisplay(0);
    QApplication a(argc, argv);
    hiveObliquePhotography::QTInterface::QTInterface w;
    w.show();
    return a.exec();
}
