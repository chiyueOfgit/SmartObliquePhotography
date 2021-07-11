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
#include <qpainter.h>

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

CQTInterface::CQTInterface(QWidget * vParent)
    : QMainWindow(vParent)
{
    ui.setupUi(this);
}

CQTInterface::~CQTInterface()
{
}

void CQTInterface::init()
{
    {
        Visualization::hiveGetVisualizationConfig(m_pVisualizationConfig);
    }

    __connectSignals();
    __initialResourceSpaceDockWidget();
    __initialWorkSpaceDockWidget();
    __initialMessageDockWidget();
    __parseConfigFile();
}

void CQTInterface::__connectSignals()
{
    QObject::connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(onActionOpen()));
    QObject::connect(ui.actionSave, SIGNAL(triggered()), this, SLOT(onActionSave()));
    QObject::connect(ui.actionPointPicking, SIGNAL(triggered()), this, SLOT(onActionPointPicking()));
    QObject::connect(ui.actionUpdate, SIGNAL(triggered()), this, SLOT(onActionDiscardAndRecover()));
    QObject::connect(ui.actionDelete, SIGNAL(triggered()), this, SLOT(onActionDelete()));
    QObject::connect(ui.actionRubber, SIGNAL(triggered()), this, SLOT(onActionRubber()));
    QObject::connect(ui.actionBrush, SIGNAL(triggered()), this, SLOT(onActionBrush()));
    QObject::connect(ui.resourceSpaceTreeView, SIGNAL(doubleClicked(QModelIndex)), this, SLOT(onResourceSpaceItemDoubleClick(QModelIndex)));
    QObject::connect(ui.actionInstructions, SIGNAL(triggered()), this, SLOT(onActionInstructions()));
    QObject::connect(ui.actionOutlierDetection, SIGNAL(triggered()), this, SLOT(onActionOutlierDetection()));
}

void CQTInterface::__initialVTKWidget()
{
    auto pViewer = static_cast<pcl::visualization::PCLVisualizer*>(Visualization::hiveGetPCLVisualizer());
    ui.VTKWidget->SetRenderWindow(pViewer->getRenderWindow());
    pViewer->setupInteractor(ui.VTKWidget->GetInteractor(), ui.VTKWidget->GetRenderWindow());
    ui.VTKWidget->update();
}

void CQTInterface::__initialResourceSpaceDockWidget()
{
    m_pResourceSpaceStandardItemModels = new QStandardItemModel(ui.resourceSpaceTreeView);
    ui.resourceSpaceTreeView->setModel(m_pResourceSpaceStandardItemModels);
    m_pResourceSpaceStandardItemModels->setHorizontalHeaderLabels(QStringList() << QStringLiteral(""));

    CQTInterface::__initialDockWidgetTitleBar(ui.resourceSpaceDockWidget, "Resource Space");
}

void CQTInterface::__initialWorkSpaceDockWidget()
{
    m_pWorkSpaceStandardItemModels = new QStandardItemModel(ui.workSpaceTreeView);
    ui.workSpaceTreeView->setModel(m_pWorkSpaceStandardItemModels);
    m_pWorkSpaceStandardItemModels->setHorizontalHeaderLabels(QStringList() << QStringLiteral(""));

    CQTInterface::__initialDockWidgetTitleBar(ui.workSpaceDockWidget, "Work Space");
}

void CQTInterface::__initialMessageDockWidget()
{
    CQTInterface::__initialDockWidgetTitleBar(ui.messageDockWidget, "Output Message");
}

void CQTInterface::__initialDockWidgetTitleBar(QDockWidget* vParentWidget, const std::string& vTitleBarText)
{
    auto BackgroundColor = *CQInterfaceConfig::getInstance()->getAttribute<std::tuple<int, int, int, int>>("DOCKWIDGETTITLEBAR_BACKGROUNDCOLOR");
    auto FontColor = *CQInterfaceConfig::getInstance()->getAttribute<std::tuple<int, int, int, int>>("DOCKWIDGETTITLEBAR_FONTCOLOR");
    auto FontSize = *CQInterfaceConfig::getInstance()->getAttribute<int>("DOCKWIDGETTITLEBAR_FONTSIZE");

    QTDockWidgetTitleBar* dockWidgetTitleBar = new QTDockWidgetTitleBar(vParentWidget);
    dockWidgetTitleBar->setAttr(QColor(std::get<0>(BackgroundColor), std::get<1>(BackgroundColor), std::get<2>(BackgroundColor), std::get<3>(BackgroundColor)),
        QColor(std::get<0>(FontColor), std::get<1>(FontColor), std::get<2>(FontColor), std::get<3>(FontColor)), FontSize, QString::fromStdString(vTitleBarText));
    vParentWidget->setTitleBarWidget(dockWidgetTitleBar);
}

void CQTInterface::__initialSlider(const QStringList& vFilePathList)
{
    const std::string& FirstCloudFilePath = vFilePathList[0].toStdString();
    auto FileCloudFileName = CQTInterface::__getFileName(FirstCloudFilePath);

    auto pSubWindow = new QMdiSubWindow(ui.VTKWidget);

    m_pPointSizeSlider = new QSlider(Qt::Horizontal);
    m_pPointSizeSlider->setMinimum(1);
    m_pPointSizeSlider->setMaximum(7);
    m_pPointSizeSlider->setSingleStep(1);
    m_pPointSizeSlider->setTickInterval(1);
    m_pPointSizeSlider->setTickPosition(QSlider::TicksAbove);
    m_pPointSizeSlider->setValue(*m_pVisualizationConfig->getAttribute<int>("POINT_SHOW_SIZE"));

    connect(m_pPointSizeSlider, &QSlider::valueChanged, [&]()
        {
            m_PointSize = m_pPointSizeSlider->value();
            auto OverwriteSuccess = m_pVisualizationConfig->overwriteAttribute("POINT_SHOW_SIZE", m_PointSize);
            auto q = *m_pVisualizationConfig->getAttribute<int>("POINT_SHOW_SIZE");
            if (OverwriteSuccess)
            {
                std::vector<std::size_t> PointLabel;
                PointCloudRetouch::hiveDumpPointLabel(PointLabel);
                Visualization::hiveRefreshVisualizer(PointLabel);
            }
        }
    );

    pSubWindow->setWidget(m_pPointSizeSlider);
    pSubWindow->resize(200, 50);                // magic
    pSubWindow->setWindowFlag(Qt::FramelessWindowHint);
    pSubWindow->show();
}

void CQTInterface::__parseConfigFile()
{
    const std::string ConfigPath = "PointCloudRetouchConfig.xml";
    m_pPointCloudRetouchConfig = new hiveObliquePhotography::PointCloudRetouch::CPointCloudRetouchConfig;
    if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, m_pPointCloudRetouchConfig) != hiveConfig::EParseResult::SUCCEED)
    {
        _HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
        return;
    }
}

bool CQTInterface::__addResourceSpaceCloudItem(const std::string& vFilePath)
{
    const auto& FileName = CQTInterface::__getFileName(vFilePath);

    QStandardItem* StandardItem = new QStandardItem(QString::fromStdString(FileName));
    StandardItem->setCheckable(true);
    StandardItem->setCheckState(Qt::Checked);
    StandardItem->setEditable(false);
    m_pResourceSpaceStandardItemModels->appendRow(StandardItem);

    m_CurrentCloud = FileName;
    CQTInterface::__messageDockWidgetOutputText(QString::fromStdString(vFilePath + " is opened."));

    return true;
}

bool CQTInterface::__messageDockWidgetOutputText(QString vString)
{
    QDateTime CurrentDateTime = QDateTime::currentDateTime();
    QString CurrentDateTimeString = CurrentDateTime.toString("[yyyy-MM-dd hh:mm:ss] ");
    ui.textBrowser->append(CurrentDateTimeString + vString);

    return true;
}

std::string CQTInterface::__getFileName(const std::string& vFilePath)
{
    return vFilePath.substr(vFilePath.find_last_of('/') + 1, vFilePath.find_last_of('.') - vFilePath.find_last_of('/') - 1);
}

std::string CQTInterface::__getDirectory(const std::string& vFilePath)
{
    return vFilePath.substr(0, vFilePath.find_last_of('/'));
}

void CQTInterface::onActionPointPicking()
{
    if (m_pCloud)
    {
        if (ui.actionPointPicking->isChecked())
        {
            m_pPointPickingDockWidget = new CSliderSizeDockWidget(ui.VTKWidget, m_pVisualizationConfig);
            m_pPointPickingDockWidget->setWindowTitle(QString("Point Picking"));
            m_pPointPickingDockWidget->show();
            CQTInterface::__messageDockWidgetOutputText(QString::fromStdString("Switch to select mode."));
        }
        else
        {
            m_pPointPickingDockWidget->close();
            delete m_pPointPickingDockWidget;

            std::vector<std::size_t> PointLabel;
            PointCloudRetouch::hiveDumpPointLabel(PointLabel);
            Visualization::hiveRefreshVisualizer(PointLabel);
            CQTInterface::__messageDockWidgetOutputText(QString::fromStdString("Switch to view mode."));
        }

        if (m_pVisualizationConfig)
            m_pVisualizationConfig->overwriteAttribute("CIRCLE_MODE", ui.actionPointPicking->isChecked());
    }

}

void CQTInterface::onActionOpen()
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

    m_pCloud = hiveObliquePhotography::hiveInitPointCloudScene(FilePathSet);

    if (m_pCloud == nullptr)
        FileOpenSuccessFlag = false;

    if (FileOpenSuccessFlag)
    {
        m_DirectoryOpenPath = CQTInterface::__getDirectory(FilePathSet.back());

        auto config = m_pPointCloudRetouchConfig->getSubconfigAt(0);
        auto num = config->getNumSubconfig();

        PointCloudRetouch::hiveInit(m_pCloud, m_pPointCloudRetouchConfig);
        Visualization::hiveInitVisualizer(m_pCloud, true);
        //Visualization::hiveRegisterQTLinker(new CQTLinker(this));
        CQTInterface::__initialVTKWidget();
        std::vector<std::size_t> PointLabel;
        PointCloudRetouch::hiveDumpPointLabel(PointLabel);
        Visualization::hiveRefreshVisualizer(PointLabel, true);
        CQTInterface::__initialSlider(FilePathList);
        
        if (FilePathSet.size() == 1)
        {
            CQTInterface::__addResourceSpaceCloudItem(FilePathSet[0]);
        }                                                                           
        else
        {
            m_SceneIndex++;
            CQTInterface::__addResourceSpaceCloudItem("Scene " + std::to_string(m_SceneIndex));
        }
    }
}

void CQTInterface::onActionSave()
{
    const auto& FilePath = QFileDialog::getSaveFileName(this, tr("Save PointCloud"), ".", tr("Save PointCloud files(*.pcd)")).toStdString();

    PointCloud_t::Ptr pCloud(new PointCloud_t);
    PointCloudRetouch::hiveSave(pCloud);
    if (hiveObliquePhotography::hiveSavePointCloudScene(*pCloud, FilePath))
        CQTInterface::__messageDockWidgetOutputText(QString::fromStdString("Save scene successfully"));
    else
        CQTInterface::__messageDockWidgetOutputText(QString::fromStdString("Scene is not saved"));
}

void CQTInterface::onActionRubber()
{
    if (m_pVisualizationConfig)
    {
        m_pVisualizationConfig->overwriteAttribute("RUBBER_MODE", ui.actionRubber->isChecked());
    }
}

void CQTInterface::onActionBrush()
{
    
}

void CQTInterface::onActionOutlierDetection()
{
    if (m_pCloud)
    {
        PointCloudRetouch::hiveRemoveOutlier();
        std::vector<std::size_t> PointLabel;
        PointCloudRetouch::hiveDumpPointLabel(PointLabel);
        Visualization::hiveRefreshVisualizer(PointLabel);
    }
}

void CQTInterface::onActionDiscardAndRecover()
{
    static int i = 1;
    if (i++ % 2)
        PointCloudRetouch::hiveDiscardUnwantedPoints();
    else
        PointCloudRetouch::hiveRecoverDiscardPoints2Unwanted();

    std::vector<std::size_t> PointLabel;
    PointCloudRetouch::hiveDumpPointLabel(PointLabel);
    Visualization::hiveRefreshVisualizer(PointLabel);
}

void CQTInterface::onActionDelete()
{
    PointCloudRetouch::hiveClearMarkerResult();

    std::vector<std::size_t> PointLabel;
    PointCloudRetouch::hiveDumpPointLabel(PointLabel);
    Visualization::hiveRefreshVisualizer(PointLabel);
}

void CQTInterface::onActionInstructions()
{
    m_pInstructionsDialog = new CInstructionsDialog(this);
    m_pInstructionsDialog->exec();
}

void CQTInterface::onResourceSpaceItemDoubleClick(QModelIndex)
{
    Visualization::hiveResetVisualizer(m_pCloud, true);
    __initialVTKWidget();
    std::vector<std::size_t> PointLabel;
    PointCloudRetouch::hiveDumpPointLabel(PointLabel);
    Visualization::hiveRefreshVisualizer(PointLabel);

}

void CQTInterface::keyPressEvent(QKeyEvent* vEvent)
{
    switch (vEvent->key())
    {
    case Qt::Key_Escape:
        //__messageDockWidgetOutputText(QString("esc"));
        this->close();
        break;
    default:
        break;
    }
}

void CQTInterface::closeEvent(QCloseEvent* vEvent)
{
    QDialog* ExitDialog = new QDialog(this);
    ExitDialog->setWindowFlag(Qt::FramelessWindowHint);
    ExitDialog->deleteLater();
    ExitDialog->resize(200, 100);
    QLabel* Label = new QLabel(ExitDialog);
    Label->setText("Are you sure?");

    Label->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    QPushButton* YesButton = new QPushButton(ExitDialog);
    QPushButton* NoButton = new QPushButton(ExitDialog);
    YesButton->setStyleSheet("QPushButton{color: black;min-width:75px;max-width:75px;min-height:20px;border:1px solid white;border-radius:5px;}"
        "QPushButton:hover{background-color: #2292DD;border-color: #000000;color:rgb(255, 255, 255);}"
        "QPushButton:pressed{background-color: #111111;border-color: #333333;color: blue;}");
    NoButton->setStyleSheet("QPushButton{color: black;min-width:75px;max-width:75px;min-height:20px;border:1px solid white;border-radius:5px;}"
        "QPushButton:hover{background-color: #2292DD;border-color: #000000;color:rgb(255, 255, 255);}"
        "QPushButton:pressed{background-color: #111111;border-color: #333333;color: blue;}");
    YesButton->setText("Yes");
    NoButton->setText("No");
    YesButton->setMaximumWidth(100);
    NoButton->setMaximumWidth(100);

    QObject::connect(YesButton, &QPushButton::clicked, [=]()
        {
            ExitDialog->done(1);
        });
    QObject::connect(NoButton, &QPushButton::clicked, [=]()
        {
            ExitDialog->done(0);
        });

    QHBoxLayout* hLayout = new QHBoxLayout();
    hLayout->setSpacing(5);
    hLayout->addStretch();
    hLayout->addWidget(YesButton);
    hLayout->addWidget(NoButton);
    QVBoxLayout* v = new QVBoxLayout();
    v->addWidget(Label);
    v->addItem(hLayout);
    ExitDialog->setLayout(v);
    if (1 == ExitDialog->exec())
    {
        vEvent->accept();
    }
    else
    {
        vEvent->ignore();
    }
}