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

#include "QTDockWidgetTitleBar.h"
#include "SliderSizeDockWidget.h"
#include "GrowingOptionsSettingDialog.h"
#include "BinaryOptionsSettingDialog.h"
#include "QTInterfaceConfig.h"
#include "VisualizationConfig.h"
#include "ObliquePhotographyDataInterface.h"
#include "AutoRetouchInterface.h"
#include "VisualizationInterface.h"
#include "QTLinker.h"

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

using namespace hiveObliquePhotography::QTInterface;

QTInterface::QTInterface(QWidget * vParent)
    : QMainWindow(vParent)
{
    {
        Visualization::hiveGetVisualizationConfig(m_pVisualizationConfig);
        AutoRetouch::hiveGetAutoRetouchConfig(m_pAutoRetouchConfig );
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
    QObject::connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(onActionOpen()));
    QObject::connect(ui.actionSave, SIGNAL(triggered()), this, SLOT(onActionSave()));
    QObject::connect(ui.actionSetting, SIGNAL(triggered()), this, SLOT(onActionSetting()));
    QObject::connect(ui.actionDelete, SIGNAL(triggered()), this, SLOT(onActionResetSelectStatus()));
    QObject::connect(ui.actionUpdate, SIGNAL(triggered()), this, SLOT(onActionUpdate()));
    QObject::connect(ui.actionCompositeBinary, SIGNAL(triggered()), this, SLOT(onActionCompositeBinary()));
    QObject::connect(ui.actionBinary, SIGNAL(triggered()), this, SLOT(onActionBinary()));
    QObject::connect(ui.actionRegionGrowing, SIGNAL(triggered()), this, SLOT(onActionRegionGrowing()));
    QObject::connect(ui.actionRubber, SIGNAL(triggered()), this, SLOT(onActionRubber()));
    QObject::connect(ui.actionBrush, SIGNAL(triggered()), this, SLOT(onActionBrush()));
    QObject::connect(ui.resourceSpaceTreeView, SIGNAL(doubleClicked(QModelIndex)), this, SLOT(onResourceSpaceItemDoubleClick(QModelIndex)));
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
    m_pPointSizeSlider->setValue(*m_pVisualizationConfig->getAttribute<int>("POINT_SHOW_SIZE"));

    connect(m_pPointSizeSlider, &QSlider::valueChanged, [&]()
        {
            m_PointSize = m_pPointSizeSlider->value();
            auto OverwriteSuccess = m_pVisualizationConfig->overwriteAttribute("POINT_SHOW_SIZE", m_PointSize);
            auto q = *m_pVisualizationConfig->getAttribute<int>("POINT_SHOW_SIZE");
            if (OverwriteSuccess)
                Visualization::hiveRefreshVisualizer();
        }
    );

    pSubWindow->setWidget(m_pPointSizeSlider);
    pSubWindow->resize(200, 50);                // magic
    pSubWindow->setWindowFlag(Qt::FramelessWindowHint);
    pSubWindow->show();
}

void QTInterface::__setActionsMutex()
{
    if (ui.actionRegionGrowing->isChecked())
    {
        ui.actionBinary->setChecked(false);
        ui.actionCompositeBinary->setChecked(false);
    }
    else if (ui.actionBinary->isChecked())
    {
        ui.actionRegionGrowing->setChecked(false);
        ui.actionCompositeBinary->setChecked(false);
    }
    else if (ui.actionCompositeBinary->isChecked())
    {
        ui.actionRegionGrowing->setChecked(false);
        ui.actionBinary->setChecked(false);
    }
}

void QTInterface::__setActionsEnabled()
{
    QList<QAction*> ObjectList = ui.mainToolBar->actions();
    for (auto Action : ObjectList)
        Action->setEnabled(true);
}

template <class T>
bool QTInterface::__readConfigFile(const std::string& vFileName, T* vInstance)
{
    if (hiveConfig::hiveParseConfig(vFileName, hiveConfig::EConfigType::XML, vInstance) != hiveConfig::EParseResult::SUCCEED)
    {
        QTInterface::__messageDockWidgetOutputText(QString::fromStdString("Failed to parse config file " + vFileName) + ".");
        return false;
    }
    else
    {
        QTInterface::__messageDockWidgetOutputText(QString::fromStdString("Succeed to parse config file " + vFileName) + ".");
        return true;
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

bool QTInterface::__deleteResourceSpaceCloudItem(const std::string& vFilePath)
{
    auto FileName = m_pResourceSpaceStandardItemModels->findItems(QString::fromStdString(QTInterface::__getFileName(vFilePath)));
    if (!FileName.empty())
    {
        auto row = FileName.first()->index().row();
        m_pResourceSpaceStandardItemModels->removeRow(row);

        QTInterface::__messageDockWidgetOutputText(QString::fromStdString(vFilePath + "is deleted."));
        return true;
    }
    else
    {
        QTInterface::__messageDockWidgetOutputText(QString::fromStdString(vFilePath + "is not deleted."));
        return false;
    }
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
        /*// check if file is opened repeatedly 
        if (std::find(m_FilePathList.begin(), m_FilePathList.end(), FilePathString) == m_FilePathList.end())
        {
            m_FilePathList.push_back(FilePathString);
            FilePathSet.push_back(FilePathString);
        }
        else
            QTInterface::__MessageDockWidgetOutputText(QString::fromStdString(FilePathString + " has been opened yet! It won't be loaded again!"));*/
        FilePathSet.push_back(FilePathString);
    }

    if (FilePathSet.empty())
        return;

    auto pCloud = hiveObliquePhotography::hiveInitPointCloudScene(FilePathSet);
    m_pCloud = pCloud;

    _ASSERT(pCloud);
    if (pCloud == nullptr)
        FileOpenSuccessFlag = false;

    if (FileOpenSuccessFlag)
    {
        m_DirectoryOpenPath = QTInterface::__getDirectory(FilePathSet.back());
        AutoRetouch::hiveInitPointCloudScene(pCloud);
        Visualization::hiveInitVisualizer(pCloud);
        Visualization::hiveRegisterQTLinker(new CQTLinker(this));
        QTInterface::__initialVTKWidget();
        Visualization::hiveRefreshVisualizer(true);
        QTInterface::__initialSlider(FilePathList);
        QTInterface::__setActionsEnabled();

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

void QTInterface::onActionSave()
{
    const auto& FilePath = QFileDialog::getSaveFileName(this, tr("Save PointCloud"), ".", tr("Save PointCloud files(*.pcd)")).toStdString();
	
    PointCloud_t::Ptr pCloud(new pcl::PointCloud<pcl::PointSurfel>);
    hiveObliquePhotography::AutoRetouch::hiveGetPointCloudForSave(pCloud);
    if(hiveObliquePhotography::hiveSavePointCloudScene(*pCloud, FilePath))
          QTInterface::__messageDockWidgetOutputText(QString::fromStdString("Save scene successfully"));
    else
        QTInterface::__messageDockWidgetOutputText(QString::fromStdString("Scene is not saved"));
}

void QTInterface::onActionSetting()
{
    //std::shared_ptr<CGrowingOptionsSettingDialog> pDisplayOptionsSettingDialog = std::make_shared<CGrowingOptionsSettingDialog>(this);
    //pDisplayOptionsSettingDialog->show();
    //pDisplayOptionsSettingDialog->exec();
    ui.actionSetting->setChecked(false);
}

void QTInterface::onActionResetSelectStatus()
{
    AutoRetouch::hiveResetSceneSelectStatus();
    Visualization::hiveRefreshVisualizer();
    QTInterface::__messageDockWidgetOutputText(QString::fromStdString("All operations have been reset."));
}

void QTInterface::onActionUpdate()
{
    AutoRetouch::hiveExecuteCompositeBinaryClassifier();
    Visualization::hiveRefreshVisualizer();
    QTInterface::__messageDockWidgetOutputText(QString::fromStdString("Composite binary algorithm run successfully."));
}

void QTInterface::onActionCompositeBinary()
{
    ui.actionBinary->setChecked(false);
    ui.actionRegionGrowing->setChecked(false);

    if (m_pBinaryOptionsSettingDialog)
    {
        m_pBinaryOptionsSettingDialog->close();
        _SAFE_DELETE(m_pBinaryOptionsSettingDialog);
    }

    if (ui.actionCompositeBinary->isChecked())
    {
        if (m_pGrowingOptionsSettingDialog)
        {
            m_pGrowingOptionsSettingDialog->close();
            _SAFE_DELETE(m_pGrowingOptionsSettingDialog);
        }

        m_pBinaryOptionsSettingDialog = new CBinaryOptionsSettingDialog(this);
        m_pBinaryOptionsSettingDialog->show();
        m_pBinaryOptionsSettingDialog->exec();
    }

    m_pVisualizationConfig->overwriteAttribute("BINARY_MODE", ui.actionCompositeBinary->isChecked());

}

void QTInterface::onActionBinary()
{
    ui.actionCompositeBinary->setChecked(false);
    ui.actionRegionGrowing->setChecked(false);
}

void QTInterface::onActionRegionGrowing()
{
    ui.actionCompositeBinary->setChecked(false);
    ui.actionBinary->setChecked(false);

    if (m_pGrowingOptionsSettingDialog)
    {
        m_pGrowingOptionsSettingDialog->close();
        _SAFE_DELETE(m_pGrowingOptionsSettingDialog);
    }

    if (ui.actionRegionGrowing->isChecked())
    {
        if (m_pBinaryOptionsSettingDialog)
        {
            m_pBinaryOptionsSettingDialog->close();
            _SAFE_DELETE(m_pBinaryOptionsSettingDialog);
        }

        m_pGrowingOptionsSettingDialog = new CGrowingOptionsSettingDialog(this);
        m_pGrowingOptionsSettingDialog->show();
        m_pGrowingOptionsSettingDialog->exec();
    }

    m_pVisualizationConfig->overwriteAttribute("BINARY_MODE", !ui.actionRegionGrowing->isChecked());
}

void QTInterface::onActionRubber()
{
    if (ui.actionRubber->isChecked())
    {
        if (m_pBrushSizeDockWidget != nullptr)
            m_pBrushSizeDockWidget->close();
        m_pRubberSizeDockWidget = new CSliderSizeDockWidget(ui.VTKWidget);
        m_pRubberSizeDockWidget->setWindowTitle(QString("Rubber Size"));
        m_pRubberSizeDockWidget->show();
        QTInterface::__messageDockWidgetOutputText(QString::fromStdString("Switch to rubber."));
    }
    else
        m_pRubberSizeDockWidget->close();

    bool bRubber = ui.actionRubber->isChecked();
    ui.actionBrush->setChecked(false);
    m_pVisualizationConfig->overwriteAttribute("LINE_MODE", bRubber);
    Visualization::hiveSetLineMode(bRubber);
    m_pVisualizationConfig->overwriteAttribute("UNWANTED_MODE", !bRubber);
}

void QTInterface::onActionBrush()
{
    if (ui.actionBrush->isChecked())
    {
        if (m_pRubberSizeDockWidget != nullptr)
            m_pRubberSizeDockWidget->close();
        m_pBrushSizeDockWidget = new CSliderSizeDockWidget(ui.VTKWidget);
        m_pBrushSizeDockWidget->setWindowTitle(QString("Brush Size"));
        m_pBrushSizeDockWidget->show();
        QTInterface::__messageDockWidgetOutputText(QString::fromStdString("Switch to brush."));
    }
    else
        m_pBrushSizeDockWidget->close();

    bool bBrush = ui.actionBrush->isChecked();
    ui.actionRubber->setChecked(false);
    m_pVisualizationConfig->overwriteAttribute("LINE_MODE", bBrush);
    Visualization::hiveSetLineMode(bBrush);
    m_pVisualizationConfig->overwriteAttribute("UNWANTED_MODE", bBrush);
}

void QTInterface::onResourceSpaceItemDoubleClick(const QModelIndex& vIndex)
{
    Visualization::hiveResetVisualizer(m_pCloud, true);
    __initialVTKWidget();
}

void QTInterface::closeEvent(QCloseEvent* vEvent)
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
