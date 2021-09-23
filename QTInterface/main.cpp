#include "pch.h"

#include <vtkAutoInit.h>

#include "QTInterface.h"
#include <QtWidgets/QApplication>
#include <vtkOutputWindow.h>

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);
VTK_MODULE_INIT(vtkRenderingFreeType);

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
