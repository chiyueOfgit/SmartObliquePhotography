#include "pch.h"
#include "QTInterface.h"
#include <QtWidgets/QApplication>
#include <vtkOutputWindow.h>

int main(int argc, char *argv[])
{
    vtkOutputWindow::SetGlobalWarningDisplay(0);
    QApplication Application(argc, argv);
    hiveObliquePhotography::QTInterface::CQTInterface Window;
    Window.setWindowTitle("SmartObliquePhotography");
    Window.setWindowState(Qt::WindowMaximized);
    Window.setWindowIcon(QIcon("Icon/tree.png"));
    Window.init();
    Window.show();
    return Application.exec();
}
