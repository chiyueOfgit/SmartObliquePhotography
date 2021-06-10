#include "pch.h"
#include "QTInterface.h"
#include "QTDockWidgetTitleBar.h"
#include <QtWidgets/qmdisubwindow.h>
#include <QSlider>
#include <QtWidgets/QFileDialog>
#include <QStandardItem>
#include <qobject.h>
#include <vtkRenderWindow.h>
#include <QDateTime>
#include <QColor>
#include <qlabel.h>
#include <iostream>
#include <string>
#include "ui_DisplayOptionsSettingDialog.h"
#include "DisplayOptionsSettingDialog.h"
#include <QMainWindow>
#include <vtkAutoInit.h>
#include "ObliquePhotographyDataInterface.h"
#include "AutoRetouchInterface.h"
#include "VisualizationInterface.h"

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

using namespace hiveObliquePhotography::QTInterface;

QTInterface::QTInterface(QWidget * vParent)
    : QMainWindow(vParent)
{
    ui.setupUi(this);

    __connectSignals();

}

QTInterface::~QTInterface()
{
}

void QTInterface::__connectSignals()
{
    QObject::connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(onActionOpen()));
}

void QTInterface::onActionOpen()
{
    QStringList FileNameList = QFileDialog::getOpenFileNames(this, tr("Open PointCloud"), ".", tr("Open PointCloud files(*.pcd)"));
    std::vector<std::string> FileNameSet;

    foreach(QString str, FileNameList)
    {
        FileNameSet.push_back(str.toStdString());
    }
    
    auto pCloud = hiveObliquePhotography::hiveInitPointCloudScene(FileNameSet);
    if (pCloud)
    {
        AutoRetouch::hiveInitPointCloudScene(pCloud);
        Visualization::hiveInitVisualizer(pCloud);
        __initialVTKWidget();
        Visualization::hiveRefreshVisualizer();
    }
}

void QTInterface::__initialVTKWidget()
{
    auto pViewer = static_cast<pcl::visualization::PCLVisualizer*>(Visualization::hiveGetPCLVisualizer());
    ui.VTKWidget->SetRenderWindow(pViewer->getRenderWindow());
    pViewer->setupInteractor(ui.VTKWidget->GetInteractor(), ui.VTKWidget->GetRenderWindow());
    ui.VTKWidget->update();
}
