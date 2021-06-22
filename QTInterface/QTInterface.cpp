#include "pch.h"
#include "QTInterface.h"
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
#include <QtWidgets/qmainwindow.h>
#include <QMainWindow>
#include <vtkAutoInit.h>
#include <common/ConfigCommon.h>
#include <common/ConfigInterface.h>
#include <algorithm>
#include <tuple>
#include <typeinfo>
#include <qpushbutton.h>

#include "QTDockWidgetTitleBar.h"
#include "QTInterfaceConfig.h"
#include "SliderSizeDockWidget.h"

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

using namespace hiveObliquePhotography::QTInterface;

QTInterface::QTInterface(QWidget * vParent)
    : QMainWindow(vParent)
{
    {
        //Visualization::hiveGetVisualizationConfig(m_pVisualizationConfig);
        //AutoRetouch::hiveGetAutoRetouchConfig(m_pAutoRetouchConfig );
    }

    ui.setupUi(this);

    __connectSignals();
    __initialResourceSpaceDockWidget();
    __initialWorkSpaceDockWidget();
    __initialMessageDockWidget();
}

QTInterface::~QTInterface()
{
}

void QTInterface::__connectSignals()
{
    QObject::connect(ui.actionPointPicking, SIGNAL(triggered()), this, SLOT(onActionPointPicking()));
}

void QTInterface::__initialVTKWidget()
{
}

void QTInterface::__initialResourceSpaceDockWidget()
{
    m_pResourceSpaceStandardItemModels = new QStandardItemModel(ui.resourceSpaceTreeView);
    ui.resourceSpaceTreeView->setModel(m_pResourceSpaceStandardItemModels);
    m_pResourceSpaceStandardItemModels->setHorizontalHeaderLabels(QStringList() << QStringLiteral(""));

    QTInterface::__initialDockWidgetTitleBar(ui.resourceSpaceDockWidget, "Resource Space");
}

void QTInterface::__initialWorkSpaceDockWidget()
{
    m_pWorkSpaceStandardItemModels = new QStandardItemModel(ui.workSpaceTreeView);
    ui.workSpaceTreeView->setModel(m_pWorkSpaceStandardItemModels);
    m_pWorkSpaceStandardItemModels->setHorizontalHeaderLabels(QStringList() << QStringLiteral(""));

    QTInterface::__initialDockWidgetTitleBar(ui.workSpaceDockWidget, "Work Space");
}

void QTInterface::__initialMessageDockWidget()
{
    QTInterface::__initialDockWidgetTitleBar(ui.messageDockWidget, "Output Message");
}

void QTInterface::__initialDockWidgetTitleBar(QDockWidget* vParentWidget, const std::string& vTitleBarText)
{
    auto BackgroundColor = *CQInterfaceConfig::getInstance()->getAttribute<std::tuple<int, int, int, int>>("DOCKWIDGETTITLEBAR_BACKGROUNDCOLOR");
    auto FontColor = *CQInterfaceConfig::getInstance()->getAttribute<std::tuple<int, int, int, int>>("DOCKWIDGETTITLEBAR_FONTCOLOR");
    auto FontSize = *CQInterfaceConfig::getInstance()->getAttribute<int>("DOCKWIDGETTITLEBAR_FONTSIZE");

    QTDockWidgetTitleBar* dockWidgetTitleBar = new QTDockWidgetTitleBar(vParentWidget);
    dockWidgetTitleBar->setAttr(QColor(std::get<0>(BackgroundColor), std::get<1>(BackgroundColor), std::get<2>(BackgroundColor), std::get<3>(BackgroundColor)),
        QColor(std::get<0>(FontColor), std::get<1>(FontColor), std::get<2>(FontColor), std::get<3>(FontColor)), FontSize, QString::fromStdString(vTitleBarText));
    vParentWidget->setTitleBarWidget(dockWidgetTitleBar);
}

void QTInterface::__initialSlider(const QStringList& vFilePathList)
{
    const std::string& FirstCloudFilePath = vFilePathList[0].toStdString();
    auto FileCloudFileName = QTInterface::__getFileName(FirstCloudFilePath);

    auto pSubWindow = new QMdiSubWindow(ui.VTKWidget);

    m_pPointSizeSlider = new QSlider(Qt::Horizontal);
    m_pPointSizeSlider->setMinimum(1);
    m_pPointSizeSlider->setMaximum(7);
    m_pPointSizeSlider->setSingleStep(1);
    m_pPointSizeSlider->setTickInterval(1);
    m_pPointSizeSlider->setTickPosition(QSlider::TicksAbove);

    pSubWindow->setWidget(m_pPointSizeSlider);
    pSubWindow->resize(200, 50);                // magic
    pSubWindow->setWindowFlag(Qt::FramelessWindowHint);
    pSubWindow->show();
}

bool QTInterface::__messageDockWidgetOutputText(QString vString)
{
    QDateTime CurrentDateTime = QDateTime::currentDateTime();
    QString CurrentDateTimeString = CurrentDateTime.toString("[yyyy-MM-dd hh:mm:ss] ");
    ui.textBrowser->append(CurrentDateTimeString + vString);

    return true;
}

std::string QTInterface::__getFileName(const std::string& vFilePath)
{
    return vFilePath.substr(vFilePath.find_last_of('/') + 1, vFilePath.find_last_of('.') - vFilePath.find_last_of('/') - 1);
}

std::string QTInterface::__getDirectory(const std::string& vFilePath)
{
    return vFilePath.substr(0, vFilePath.find_last_of('/'));
}

void QTInterface::onActionPointPicking()
{

    //if (m_pBrushSizeDockWidget != nullptr)
    /*m_pBrushSizeDockWidget->close();
    m_pRubberSizeDockWidget = new CSliderSizeDockWidget(ui.VTKWidget);
    m_pRubberSizeDockWidget->setWindowTitle(QString("Rubber Size"));
    m_pRubberSizeDockWidget->show();
    QTInterface::__messageDockWidgetOutputText(QString::fromStdString("Switch to rubber."));*/

    QDockWidget* m_pPointPickingDockWidget = new CSliderSizeDockWidget(ui.VTKWidget);
    m_pPointPickingDockWidget->setWindowTitle(QString("Point Picking"));
    m_pPointPickingDockWidget->show();
    QTInterface::__messageDockWidgetOutputText(QString::fromStdString("Switch to point picking.")); 
}