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
#include <algorithm>
#include <tuple>
#include <typeinfo>
#include <qpushbutton.h>
#include <qcursor.h>
#include <qevent.h>

#include "QTDockWidgetTitleBar.h"
#include "QTInterfaceConfig.h"
#include "SliderSizeDockWidget.h"
#include "ObliquePhotographyDataInterface.h"
#include "PointCloudRetouchInterface.h"
#include "VisualizationInterface.h"
#include "PointCloudRetouchConfig.h"

#include "pcl/io/pcd_io.h"

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

    QCursor* myCursor = new QCursor(QPixmap("Icon/pointPicking.png"), -1, -1);    //-1,-1表示热点位于图片中心
    this->setCursor(*myCursor);
    this->unsetCursor();

    //this->grabKeyboard();

    __connectSignals();
    __initialResourceSpaceDockWidget();
    __initialWorkSpaceDockWidget();
    __initialMessageDockWidget();
    __parseConfigFile();
}

QTInterface::~QTInterface()
{
}

void QTInterface::__connectSignals()
{
    QObject::connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(onActionOpen()));
    QObject::connect(ui.actionPointPicking, SIGNAL(triggered()), this, SLOT(onActionPointPicking()));
}

void QTInterface::__initialVTKWidget()
{
    auto pViewer = static_cast<pcl::visualization::PCLVisualizer*>(Visualization::hiveGetPCLVisualizer());
    ui.VTKWidget->SetRenderWindow(pViewer->getRenderWindow());
    pViewer->setupInteractor(ui.VTKWidget->GetInteractor(), ui.VTKWidget->GetRenderWindow());
    ui.VTKWidget->update();
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

void QTInterface::__parseConfigFile()
{
    const std::string ConfigPath = "PointCloudRetouchConfig.xml";
    m_pPointCloudRetouchConfig = new hiveObliquePhotography::PointCloudRetouch::CPointCloudRetouchConfig;
    if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, m_pPointCloudRetouchConfig) != hiveConfig::EParseResult::SUCCEED)
    {
        _HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
        return;
    }
}

bool QTInterface::__addResourceSpaceCloudItem(const std::string& vFilePath)
{
    const auto& FileName = QTInterface::__getFileName(vFilePath);

    QStandardItem* StandardItem = new QStandardItem(QString::fromStdString(FileName));
    StandardItem->setCheckable(true);
    StandardItem->setCheckState(Qt::Checked);
    StandardItem->setEditable(false);
    m_pResourceSpaceStandardItemModels->appendRow(StandardItem);

    m_CurrentCloud = FileName;
    QTInterface::__messageDockWidgetOutputText(QString::fromStdString(vFilePath + " is opened."));

    return true;
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
    m_pPointPickingDockWidget = new CSliderSizeDockWidget(ui.VTKWidget);
    m_pPointPickingDockWidget->setWindowTitle(QString("Point Picking"));
    m_pPointPickingDockWidget->show();
    QTInterface::__messageDockWidgetOutputText(QString::fromStdString("Switch to point picking.")); 
}

void QTInterface::onActionOpen()
{
    QStringList FilePathList = QFileDialog::getOpenFileNames(this, tr("Open PointCloud"), QString::fromStdString(m_DirectoryOpenPath), tr("Open PointCloud files(*.pcd)"));
    std::vector<std::string> FilePathSet;
    bool FileOpenSuccessFlag = true;

    if (FilePathList.empty())
        return;

    foreach(QString FilePathQString, FilePathList)
    {
        std::string FilePathString = FilePathQString.toStdString();
        FilePathSet.push_back(FilePathString);
    }

    if (FilePathSet.empty())
        return;

    PointCloud_t::Ptr pCloud(new PointCloud_t);
    pcl::io::loadPCDFile(FilePathSet.front(), *pCloud);
    //PointCloud_t::Ptr pCloud = hiveObliquePhotography::hiveInitPointCloudScene(FilePathSet);
    m_pCloud = pCloud;

    if (pCloud == nullptr)
        FileOpenSuccessFlag = false;

    if (FileOpenSuccessFlag)
    {
        m_DirectoryOpenPath = QTInterface::__getDirectory(FilePathSet.back());
        PointCloudRetouch::hiveInit(pCloud, m_pPointCloudRetouchConfig);
        Visualization::hiveInitVisualizer(pCloud, true);
        //Visualization::hiveRegisterQTLinker(new CQTLinker(this));
        QTInterface::__initialVTKWidget();
        std::vector<std::size_t> PointLabel;
        PointCloudRetouch::hiveDumpPointLabel(PointLabel);
        Visualization::hiveRefreshVisualizer(PointLabel, true);
        Visualization::hiveRunVisualizerLoop();
        QTInterface::__initialSlider(FilePathList);

        if (FilePathSet.size() == 1)
        {
            QTInterface::__addResourceSpaceCloudItem(FilePathSet[0]);
        }                                                                           
        else
        {
            m_SceneIndex++;
            QTInterface::__addResourceSpaceCloudItem("Scene " + std::to_string(m_SceneIndex));
        }
    }
}

void QTInterface::keyPressEvent(QKeyEvent* vEvent)
{
    switch (vEvent->key())
    {
    case Qt::Key_Escape:
        __messageDockWidgetOutputText(QString("esc"));
        break;
    default:
        break;
    }
}