#include "pch.h"
#include <QtWidgets/QApplication>
#include <vtkOutputWindow.h>
#include "MeshHighlight.h"

int main(int argc, char* argv[])
{
    vtkOutputWindow::SetGlobalWarningDisplay(0);
    QApplication Application(argc, argv);
    hiveObliquePhotography::MeshHighlight::CMeshHighlight Window;
    Window.init();
    Window.show();
    return Application.exec();
}