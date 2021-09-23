#include "pch.h"
#include "FeatureVisualization.h"
#include <QtWidgets/QApplication>
#include <vtkOutputWindow.h>

int main(int argc, char *argv[])
{
    vtkOutputWindow::SetGlobalWarningDisplay(0);
    QApplication Application(argc, argv);
    hiveObliquePhotography::FeatureVisualization::CFeatureVisualization Window;
    Window.init();
    Window.show();
    return Application.exec();
}
