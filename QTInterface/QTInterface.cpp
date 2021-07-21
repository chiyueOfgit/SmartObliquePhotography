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
#include "DisplayOptionsSettingDialog.h"

#include "pcl/io/pcd_io.h"

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

using namespace hiveObliquePhotography::QTInterface;

CQTInterface::CQTInterface(QWidget * vParent)
    : QMainWindow(vParent)
{
    m_UI.setupUi(this);
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
    QObject::connect(m_UI.actionOpen, SIGNAL(triggered()), this, SLOT(onActionOpen()));
    QObject::connect(m_UI.actionSave, SIGNAL(triggered()), this, SLOT(onActionSave()));
    QObject::connect(m_UI.actionPointPicking, SIGNAL(triggered()), this, SLOT(onActionPointPicking()));
    QObject::connect(m_UI.actionUpdate, SIGNAL(triggered()), this, SLOT(onActionDiscardAndRecover()));
    QObject::connect(m_UI.actionDeleteLitter , SIGNAL(triggered()), this, SLOT(onActionDeleteLitter()));
    QObject::connect(m_UI.actionDeleteBackground, SIGNAL(triggered()), this, SLOT(onActionDeleteBackground()));
    QObject::connect(m_UI.actionPrecompute, SIGNAL(triggered()), this, SLOT(onActionPrecompute()));
    QObject::connect(m_UI.actionRubber, SIGNAL(triggered()), this, SLOT(onActionRubber()));
    QObject::connect(m_UI.actionBrush, SIGNAL(triggered()), this, SLOT(onActionBrush()));
    QObject::connect(m_UI.actionSetting, SIGNAL(triggered()), this, SLOT(onActionSetting()));
    QObject::connect(m_UI.resourceSpaceTreeView, SIGNAL(doubleClicked(QModelIndex)), this, SLOT(onResourceSpaceItemDoubleClick(QModelIndex)));
    QObject::connect(m_UI.actionInstructions, SIGNAL(triggered()), this, SLOT(onActionInstructions()));
    QObject::connect(m_UI.actionOutlierDetection, SIGNAL(triggered()), this, SLOT(onActionOutlierDetection()));
}

void CQTInterface::__initialVTKWidget()
{
    auto pViewer = static_cast<pcl::visualization::PCLVisualizer*>(Visualization::hiveGetPCLVisualizer());
    m_UI.VTKWidget->SetRenderWindow(pViewer->getRenderWindow());
    pViewer->setupInteractor(m_UI.VTKWidget->GetInteractor(), m_UI.VTKWidget->GetRenderWindow());
    m_UI.VTKWidget->update();
}

void CQTInterface::__initialResourceSpaceDockWidget()
{
    m_pResourceSpaceStandardItemModels = new QStandardItemModel(m_UI.resourceSpaceTreeView);
    m_UI.resourceSpaceTreeView->setModel(m_pResourceSpaceStandardItemModels);
    m_pResourceSpaceStandardItemModels->setHorizontalHeaderLabels(QStringList() << QStringLiteral(""));

    CQTInterface::__initialDockWidgetTitleBar(m_UI.resourceSpaceDockWidget, "Resource Space");
}

void CQTInterface::__initialWorkSpaceDockWidget()
{
    m_pWorkSpaceStandardItemModels = new QStandardItemModel(m_UI.workSpaceTreeView);
    m_UI.workSpaceTreeView->setModel(m_pWorkSpaceStandardItemModels);
    m_pWorkSpaceStandardItemModels->setHorizontalHeaderLabels(QStringList() << QStringLiteral(""));

    CQTInterface::__initialDockWidgetTitleBar(m_UI.workSpaceDockWidget, "Work Space");
}

void CQTInterface::__initialMessageDockWidget()
{
    CQTInterface::__initialDockWidgetTitleBar(m_UI.messageDockWidget, "Output Message");
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

    auto pSubWindow = new QMdiSubWindow(m_UI.VTKWidget);

    m_pPointSizeSlider = new QSlider(Qt::Horizontal);
    m_pPointSizeSlider->setMinimum(1);
    m_pPointSizeSlider->setMaximum(7);
    m_pPointSizeSlider->setSingleStep(1);
    m_pPointSizeSlider->setTickInterval(1);
    m_pPointSizeSlider->setTickPosition(QSlider::TicksAbove);
    m_pPointSizeSlider->setValue(*m_pVisualizationConfig->getAttribute<double>(Visualization::POINT_SHOW_SIZE));

    connect(m_pPointSizeSlider, &QSlider::valueChanged, [&]()
        {
            m_PointSize = m_pPointSizeSlider->value();
            auto OverwriteSuccess = m_pVisualizationConfig->overwriteAttribute(Visualization::POINT_SHOW_SIZE, static_cast<double>(m_PointSize));
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
    m_pResourceSpaceStandardItemModels->removeRow(0);
    m_pResourceSpaceStandardItemModels->appendRow(StandardItem);

    m_CurrentCloud = FileName;
    CQTInterface::__messageDockWidgetOutputText(QString::fromStdString(vFilePath + " is opened."));

    return true;
}

bool CQTInterface::__messageDockWidgetOutputText(QString vString)
{
    QDateTime CurrentDateTime = QDateTime::currentDateTime();
    QString CurrentDateTimeString = CurrentDateTime.toString("[yyyy-MM-dd hh:mm:ss] ");
    m_UI.textBrowser->append(CurrentDateTimeString + vString);

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
        if (m_UI.actionPointPicking->isChecked())
        {
            m_pPointPickingDockWidget = new CSliderSizeDockWidget(m_UI.VTKWidget, m_pVisualizationConfig);
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
            m_pVisualizationConfig->overwriteAttribute(Visualization::CIRCLE_MODE, m_UI.actionPointPicking->isChecked());
    }

}

void CQTInterface::onActionOpen()
{
    QStringList FilePathList = QFileDialog::getOpenFileNames(this, tr("Open PointCloud"), QString::fromStdString(m_DirectoryOpenPath), tr("PointCloud Files(*.pcd *.ply)"));
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

        //enable ui icons
        {
            m_UI.actionPointPicking->setEnabled(true);
            m_UI.actionSave->setEnabled(true);
            m_UI.actionUpdate->setEnabled(true);
            m_UI.actionDeleteLitter->setEnabled(true);
            m_UI.actionDeleteBackground->setEnabled(true);
            m_UI.actionPrecompute->setEnabled(true);
            m_UI.actionRubber->setEnabled(true);
            m_UI.actionBrush->setEnabled(true);
            m_UI.actionSetting->setEnabled(true);
            m_UI.actionOutlierDetection->setEnabled(true);
        }

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
    const auto& FilePath = QFileDialog::getSaveFileName(this, tr("Save PointCloud"), ".", tr("PCD files(*.pcd);;""PLY files(*.ply)")).toStdString();

    PointCloud_t::Ptr pCloud(new PointCloud_t);
    PointCloudRetouch::hiveDumpPointCloudtoSave(pCloud);
    if (hiveObliquePhotography::hiveSavePointCloudScene(*pCloud, FilePath))
        CQTInterface::__messageDockWidgetOutputText(QString::fromStdString("Save scene successfully"));
    else
        CQTInterface::__messageDockWidgetOutputText(QString::fromStdString("Scene is not saved"));
}

void CQTInterface::onActionRubber()
{
    if (m_pVisualizationConfig)
    {
        m_pVisualizationConfig->overwriteAttribute("RUBBER_MODE", m_UI.actionRubber->isChecked());
    }
}

void CQTInterface::onActionBrush()
{
    
}

void CQTInterface::onActionOutlierDetection()
{
    if (m_pCloud)
    {
        PointCloudRetouch::hiveMarkIsolatedAreaAsLitter();
        std::vector<std::size_t> PointLabel;
        PointCloudRetouch::hiveDumpPointLabel(PointLabel);
        Visualization::hiveRefreshVisualizer(PointLabel);
    }
}

void CQTInterface::onActionDiscardAndRecover()
{
    static int i = 1;
    if (i++ % 2)
        PointCloudRetouch::hiveHideLitter();
    else
        PointCloudRetouch::hiveDisplayLitter();

    std::vector<std::size_t> PointLabel;
    PointCloudRetouch::hiveDumpPointLabel(PointLabel);
    Visualization::hiveRefreshVisualizer(PointLabel);
}

void CQTInterface::onActionDeleteLitter()
{
    PointCloudRetouch::hiveRecoverLitterMark();
    Visualization::hiveRemoveAllShapes();
    Visualization::hiveCancelAllHighlighting();
    std::vector<std::size_t> PointLabel;
    PointCloudRetouch::hiveDumpPointLabel(PointLabel);
    Visualization::hiveRefreshVisualizer(PointLabel);
}

void CQTInterface::onActionDeleteBackground()
{
    PointCloudRetouch::hiveRecoverBackgroundMark();
    Visualization::hiveRemoveAllShapes();
    Visualization::hiveCancelAllHighlighting();
    std::vector<std::size_t> PointLabel;
    PointCloudRetouch::hiveDumpPointLabel(PointLabel);
    Visualization::hiveRefreshVisualizer(PointLabel);
}

void CQTInterface::onActionPrecompute()
{
    PointCloudRetouch::hiveRunPrecompute(m_CurrentCloud);
    m_UI.actionPrecompute->setChecked(false);
    __messageDockWidgetOutputText(QString::fromStdString(m_CurrentCloud + "'s precompute is finished."));
}

void CQTInterface::onActionInstructions()
{
    m_pInstructionsDialog = new CInstructionsDialog(this);
    m_pInstructionsDialog->exec();
}

void CQTInterface::onActionSetting()
{
    std::shared_ptr<CDisplayOptionsSettingDialog> pDisplayOptionsSettingDialog = std::make_shared<CDisplayOptionsSettingDialog>(this);
    pDisplayOptionsSettingDialog->show();
    pDisplayOptionsSettingDialog->exec();

    __parseConfigFile();
    PointCloudRetouch::hiveInit(m_pCloud, m_pPointCloudRetouchConfig);

    Visualization::hiveRemoveAllShapes();
    Visualization::hiveCancelAllHighlighting();

    std::vector<std::size_t> PointLabel;
    PointCloudRetouch::hiveDumpPointLabel(PointLabel);
    Visualization::hiveRefreshVisualizer(PointLabel);
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