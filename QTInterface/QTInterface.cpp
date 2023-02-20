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
#include <vtkAxesActor.h>
#include <QTranslator>

#include "QTDockWidgetTitleBar.h"
#include "QTInterfaceConfig.h"
#include "SliderSizeDockWidget.h"
#include "ObliquePhotographyDataInterface.h"
#include "PointCloudRetouchInterface.h"
#include "SceneReconstructionInterface.h"
#include "VisualizationInterface.h"
#include "PointCloudRetouchConfig.h"
#include "DisplayOptionsSettingDialog.h"
#include <vtkOrientationMarkerWidget.h>

#include "pcl/io/pcd_io.h"
#include "pcl/io/ply_io.h"
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>
#include <regex>
#include <qmessagebox.h>//����һ��ǵ�ɾ��
#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

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
    m_pDisplayOptionsSettingDialog = new CDisplayOptionsSettingDialog(this, m_pDisplayOptionsSettingDialog);
    _SAFE_DELETE(m_pDisplayOptionsSettingDialog);

    __initialResourceSpaceDockWidget();
    __initialWorkSpaceDockWidget();
    __initialMessageDockWidget();
    __connectSignals();
    __parseConfigFile();
    //m_UI.mainToolBar.setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
}

void CQTInterface::__connectSignals()
{
    QObject::connect(m_UI.actionOpen, SIGNAL(triggered()), this, SLOT(onActionOpen()));
    QObject::connect(m_UI.actionSave, SIGNAL(triggered()), this, SLOT(onActionSave()));
    QObject::connect(m_UI.actionPointPicking, SIGNAL(triggered()), this, SLOT(onActionPointPicking()));
    QObject::connect(m_UI.actionAreaPicking, SIGNAL(triggered()), this, SLOT(onActionAreaPicking()));
    QObject::connect(m_UI.actionUpdate, SIGNAL(triggered()), this, SLOT(onActionDiscardAndRecover()));
    QObject::connect(m_UI.actionDeleteLitter , SIGNAL(triggered()), this, SLOT(onActionDeleteLitter()));
    QObject::connect(m_UI.actionDeleteBackground, SIGNAL(triggered()), this, SLOT(onActionDeleteBackground()));
    QObject::connect(m_UI.actionPrecompute, SIGNAL(triggered()), this, SLOT(onActionPrecompute()));
    QObject::connect(m_UI.actionRubber, SIGNAL(triggered()), this, SLOT(onActionRubber()));
    QObject::connect(m_UI.actionBrush, SIGNAL(triggered()), this, SLOT(onActionBrush()));
    QObject::connect(m_UI.actionSetting, SIGNAL(triggered()), this, SLOT(onActionSetting()));
    QObject::connect(m_UI.actionReconstruction, SIGNAL(triggered()), this, SLOT(onActionReconstruction()));
    QObject::connect(m_UI.actionOpenMesh, SIGNAL(triggered()), this, SLOT(onActionOpenMesh()));
    QObject::connect(m_UI.actionSutureMesh, SIGNAL(triggered()), this, SLOT(onActionSutureMesh()));
    QObject::connect(m_UI.actionShowMeshIndices, SIGNAL(triggered()), this, SLOT(onActionShowMeshIndices()));
    QObject::connect(m_UI.actionParameterization, SIGNAL(triggered()), this, SLOT(onActionParameterization()));
    QObject::connect(m_UI.actionBakeTexture, SIGNAL(triggered()), this, SLOT(onActionBakeTexture()));
    QObject::connect(m_UI.resourceSpaceTreeView, SIGNAL(doubleClicked(QModelIndex)), this, SLOT(onResourceSpaceItemDoubleClick(QModelIndex)));
    //QObject::connect(m_UI.resourceSpaceTreeView, SIGNAL(clicked(QModelIndex)), this, SLOT(onResourceSpaceItemClick(QModelIndex)));
    QObject::connect(m_pResourceSpaceStandardItemModels, SIGNAL(itemChanged(QStandardItem*)), this, SLOT(onResourceSpaceItemChange(QStandardItem*)));
    QObject::connect(m_UI.actionInstructions, SIGNAL(triggered()), this, SLOT(onActionInstructions()));
    QObject::connect(m_UI.actionOutlierDetection, SIGNAL(triggered()), this, SLOT(onActionOutlierDetection()));
    QObject::connect(m_UI.actionRepairHole, SIGNAL(triggered()), this, SLOT(onActionStartRepairHole()));
    QObject::connect(m_UI.actionReconstructionStitching, SIGNAL(triggered()), this, SLOT(onActionReconstructionStitching()));
    QObject::connect(m_UI.actionParameterizationBake, SIGNAL(triggered()), this, SLOT(onActionParameterizationBake()));
    QObject::connect(m_UI.actionViewFromPositiveX, SIGNAL(triggered()), this, SLOT(onChangeCameraLookStraightAxis()));
    QObject::connect(m_UI.actionViewFromNegativeX, SIGNAL(triggered()), this, SLOT(onChangeCameraLookStraightAxis()));
    QObject::connect(m_UI.actionViewFromPositiveY, SIGNAL(triggered()), this, SLOT(onChangeCameraLookStraightAxis()));
    QObject::connect(m_UI.actionViewFromNegativeY, SIGNAL(triggered()), this, SLOT(onChangeCameraLookStraightAxis()));
    QObject::connect(m_UI.actionViewFromPositiveZ, SIGNAL(triggered()), this, SLOT(onChangeCameraLookStraightAxis()));
    QObject::connect(m_UI.actionViewFromNegativeZ, SIGNAL(triggered()), this, SLOT(onChangeCameraLookStraightAxis()));
    QObject::connect(m_UI.actionAutoModeling, SIGNAL(triggered()), this, SLOT(onActionAutoModeling()));
    QObject::connect(m_UI.actionChinese, SIGNAL(triggered()), this, SLOT(onChangeLanguage()));
    QObject::connect(m_UI.actionEnglish, SIGNAL(triggered()), this, SLOT(onChangeLanguage()));
}

void CQTInterface::__initialVTKWidget()
{
    auto pViewer = static_cast<pcl::visualization::PCLVisualizer*>(Visualization::hiveGetPCLVisualizer());
    m_UI.VTKWidget->SetRenderWindow(pViewer->getRenderWindow());
    pViewer->setupInteractor(m_UI.VTKWidget->GetInteractor(), m_UI.VTKWidget->GetRenderWindow());
    __addAxesToView(pViewer->getRenderWindow()->GetInteractor(), 0.0, 0.0, 0.2, 0.2);
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
    auto BackgroundColor = CQInterfaceConfig::getInstance()->getAttribute<std::tuple<int, int, int, int>>("DOCKWIDGETTITLEBAR_BACKGROUNDCOLOR").value();
    auto FontColor = CQInterfaceConfig::getInstance()->getAttribute<std::tuple<int, int, int, int>>("DOCKWIDGETTITLEBAR_FONTCOLOR").value();
    auto FontSize = CQInterfaceConfig::getInstance()->getAttribute<int>("DOCKWIDGETTITLEBAR_FONTSIZE").value();

    QTDockWidgetTitleBar* pDockWidgetTitleBar = new QTDockWidgetTitleBar(vParentWidget);
    pDockWidgetTitleBar->setAttr(QColor(std::get<0>(BackgroundColor), std::get<1>(BackgroundColor), std::get<2>(BackgroundColor), std::get<3>(BackgroundColor)),
        QColor(std::get<0>(FontColor), std::get<1>(FontColor), std::get<2>(FontColor), std::get<3>(FontColor)), FontSize, QString::fromStdString(vTitleBarText));
    vParentWidget->setTitleBarWidget(pDockWidgetTitleBar);
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
    m_pPointSizeSlider->setValue(m_pVisualizationConfig->getAttribute<double>(Visualization::POINT_SHOW_SIZE).value());

    connect(m_pPointSizeSlider, &QSlider::valueChanged, [&]()
        {
            m_PointSize = m_pPointSizeSlider->value();
            auto OverwriteSuccess = m_pVisualizationConfig->overwriteAttribute(Visualization::POINT_SHOW_SIZE, static_cast<double>(m_PointSize));
            if (OverwriteSuccess)
                Visualization::hiveSetPointRenderSize(m_PointSize);
        }
    );

    pSubWindow->setWidget(m_pPointSizeSlider);
    pSubWindow->resize(200, 50);
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

bool CQTInterface::__addResourceSpaceCloudItem(const std::string& vFileName)
{
    QStandardItem* pStandardItem = new QStandardItem(QString::fromStdString(vFileName));
    pStandardItem->setCheckable(true);
    pStandardItem->setCheckState(Qt::Unchecked);
    pStandardItem->setEditable(false);
    m_pResourceSpaceStandardItemModels->appendRow(pStandardItem);
    m_CurrentCloud = vFileName;
    return true;
}

bool CQTInterface::__addResourceSpaceMeshItem(const std::string& vFileName)
{
    QStandardItem* pStandardItem = new QStandardItem(QString::fromStdString(vFileName));
    pStandardItem->setCheckable(true);
    pStandardItem->setCheckState(Qt::Unchecked);
    pStandardItem->setEditable(false);
    m_pResourceSpaceStandardItemModels->appendRow(pStandardItem);
    return true;
}

bool CQTInterface::__messageDockWidgetOutputText(const std::string& vText)
{
    QDateTime CurrentDateTime = QDateTime::currentDateTime();
    QString CurrentDateTimeString = CurrentDateTime.toString("[yyyy-MM-dd hh:mm:ss] ");
    m_UI.textBrowser->append(CurrentDateTimeString + QString::fromStdString(vText));

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

std::string CQTInterface::__getFileNameWithSuffix(const std::string& vFilePath)
{
    return vFilePath.substr(vFilePath.find_last_of('/') + 1, vFilePath.length());
}

std::string CQTInterface::__getSuffix(const std::string& vFilePath)
{
    return vFilePath.substr(vFilePath.find_last_of('.') + 1, vFilePath.length());
}

void hiveObliquePhotography::QTInterface::CQTInterface::__autoSutureMeshes(QStringList vFilePathList)
{
    std::vector<std::pair<std::string, std::string>> SutureSequence;
    SceneReconstruction::hiveGetSutureSequence(SutureSequence);

    std::string DirPath = __getDirectory(vFilePathList.back().toStdString()) + std::string("/Output/");
    bool LoadMeshOne = true;
    for (auto it = SutureSequence.begin(); it != SutureSequence.end(); ++it)
    {
        std::string NameOne = it->first;
        std::string NameTwo = it->second;
        CMesh MeshOne, MeshTwo;

        if (LoadMeshOne)
        {
            hiveLoadMeshModel(MeshOne, DirPath + NameOne + std::string(".obj"));
        }
        hiveLoadMeshModel(MeshTwo, DirPath + NameTwo + std::string(".obj"));

        SceneReconstruction::hiveSutureMesh(MeshOne, MeshTwo);

        //TODO: delete mesh
        hiveSaveMeshModel(MeshOne, DirPath + NameOne + std::string(".obj"));
        if ((it + 1) != SutureSequence.end() && (it + 1)->first != NameTwo)
        {
            hiveSaveMeshModel(MeshTwo, DirPath + NameTwo + std::string(".obj"));
            LoadMeshOne = true;
        }
        else
        {
            LoadMeshOne = false;
        }

        __messageDockWidgetOutputText("Suture mesh " + NameOne + " and " + NameTwo + " succeed");
    }
}

void CQTInterface::onActionPointPicking()
{
    if (!m_TileSet.TileSet.empty())
    {
        if (m_UI.actionPointPicking->isChecked())
        {
            m_UI.actionAreaPicking->setChecked(false);
            if (m_pVisualizationConfig)
                m_pVisualizationConfig->overwriteAttribute(Visualization::AREA_MODE, false);
            if (m_pAreaPickingSetting)
                _SAFE_DELETE(m_pAreaPickingSetting);

            m_pPointPickingDockWidget = new CSliderSizeDockWidget(m_UI.VTKWidget, m_pPointPickingDockWidget, m_pVisualizationConfig);
            m_pPointPickingDockWidget->setWindowTitle(QString("Point Picking"));
            m_pPointPickingDockWidget->show();
            CQTInterface::__messageDockWidgetOutputText("Switch to point picking mode.");
        }
        else
        {
            m_pPointPickingDockWidget->close();

            std::vector<std::size_t> PointLabelSet;
            PointCloudRetouch::hiveDumpPointLabel(PointLabelSet);
            Visualization::hiveRefreshVisualizer(PointLabelSet);
            CQTInterface::__messageDockWidgetOutputText("Switch to view mode.");
        }

        if (m_pVisualizationConfig)
            m_pVisualizationConfig->overwriteAttribute(Visualization::CIRCLE_MODE, m_UI.actionPointPicking->isChecked());
    }

}

void CQTInterface::onActionAreaPicking()
{
    if (!m_TileSet.TileSet.empty())
    {
        if (!m_pVisualizationConfig->getAttribute<bool>(Visualization::REPAIR_MODE).value())
        {
            if (m_UI.actionAreaPicking->isChecked())
            {
                if (!m_pAreaPickingSetting)
                {
                    m_pAreaPickingSetting = new QMdiSubWindow(m_UI.VTKWidget);
                    m_pAreaPickingSetting->resize({ 200, 50 });
                    QPoint ParentPoint = m_UI.VTKWidget->pos();
                    m_pAreaPickingSetting->move(m_UI.VTKWidget->width() - m_pAreaPickingSetting->width(), 0);
                    m_pAreaPickingCullingBox = new QCheckBox;
                    m_pAreaPickingCullingBox->setChecked(m_pVisualizationConfig->getAttribute<bool>(Visualization::AREA_PICK_CULLING).value());
                    m_pAreaPickingCullingBox->setText(QString::fromStdString("Area picking enable culling"));
                    m_pAreaPickingCullingBox->setGeometry(150, 30, 10, 0);
                    m_pAreaPickingSetting->setWidget(m_pAreaPickingCullingBox);
                    m_pAreaPickingSetting->setWindowFlag(Qt::FramelessWindowHint);
                    m_pAreaPickingSetting->show();

                    connect(m_pAreaPickingCullingBox, &QCheckBox::stateChanged, [&]()
                        {
                            m_pVisualizationConfig->overwriteAttribute(Visualization::AREA_PICK_CULLING, m_pAreaPickingCullingBox->isChecked());
                        }
                    );
                }
                
                m_UI.actionPointPicking->setChecked(false);
                if (m_pVisualizationConfig)
                    m_pVisualizationConfig->overwriteAttribute(Visualization::CIRCLE_MODE, false);
                if (m_pPointPickingDockWidget)
                    m_pPointPickingDockWidget->close();

                std::vector<std::size_t> PointLabelSet;
                PointCloudRetouch::hiveDumpPointLabel(PointLabelSet);
                Visualization::hiveRefreshVisualizer(PointLabelSet);
                CQTInterface::__messageDockWidgetOutputText("Switch to area picking mode.");

            }
            else
            {
                if (m_pAreaPickingSetting)
                    _SAFE_DELETE(m_pAreaPickingSetting);
                CQTInterface::__messageDockWidgetOutputText("Switch to view mode.");
            }
        }

        m_pVisualizationConfig->overwriteAttribute(Visualization::AREA_MODE, m_UI.actionAreaPicking->isChecked());
    }
}

void CQTInterface::onActionOpen()
{
    QStringList FilePathList = QFileDialog::getOpenFileNames(this, tr("Open PointCloud or Mesh"), QString::fromStdString(m_CloudOpenPath), tr("PointCloud Files(*.pcd *.ply);;"));
    if (FilePathList.empty())
        return;

    std::vector<std::string> FilePathSet;
    foreach(QString FilePathQString, FilePathList)
        FilePathSet.push_back(FilePathQString.toStdString());

    m_TileSet.TileSet = hiveInitPointCloudScene(FilePathSet);
    if (!m_TileSet.TileSet.empty())
    {
        m_CloudOpenPath = __getDirectory(FilePathSet.back());

        PointCloudRetouch::hiveInit(m_TileSet.TileSet, m_pPointCloudRetouchConfig);
        if (Visualization::hiveInitVisualizer(m_TileSet.TileSet, true) == false) return;
        CQTInterface::__initialVTKWidget();
        std::vector<std::size_t> PointLabel;
        PointCloudRetouch::hiveDumpPointLabel(PointLabel);
        Visualization::hiveRefreshVisualizer(PointLabel, true);
        CQTInterface::__initialSlider(FilePathList);

        //enable ui icons
        {
            m_UI.actionPointPicking->setEnabled(true);
            m_UI.actionAreaPicking->setEnabled(true);
            m_UI.actionSave->setEnabled(true);
            m_UI.actionUpdate->setEnabled(true);
            m_UI.actionDeleteLitter->setEnabled(true);
            m_UI.actionDeleteBackground->setEnabled(true);
            m_UI.actionPrecompute->setEnabled(true);
            m_UI.actionRubber->setEnabled(true);
            m_UI.actionBrush->setEnabled(true);
            m_UI.actionSetting->setEnabled(true);
            m_UI.actionOutlierDetection->setEnabled(true);
            m_UI.actionRepairHole->setEnabled(true);
            m_UI.actionReconstruction->setEnabled(true);
            m_UI.actionBakeTexture->setEnabled(true);
            m_UI.actionViewFromNegativeX->setEnabled(true);
            m_UI.actionViewFromPositiveX->setEnabled(true);
            m_UI.actionViewFromNegativeY->setEnabled(true);
            m_UI.actionViewFromPositiveY->setEnabled(true);
            m_UI.actionViewFromNegativeZ->setEnabled(true);
            m_UI.actionViewFromPositiveZ->setEnabled(true);

            m_pVisualizationConfig->overwriteAttribute(Visualization::REPAIR_MODE, false);
        }

        // remove last loaded clouds
        for (int i = 0; i < m_pResourceSpaceStandardItemModels->rowCount();)
        {
            auto Suffix = __getSuffix(m_pResourceSpaceStandardItemModels->item(i)->text().toStdString());
            if (Suffix == "ply" || Suffix == "pcd")
                m_pResourceSpaceStandardItemModels->removeRow(i);
            else
                i++;
        }

        m_TileSet.NameSet.clear();
        for (auto& Path : FilePathSet)
            m_TileSet.NameSet.push_back(__getFileNameWithSuffix(Path));
        for (auto& Name : m_TileSet.NameSet)
            CQTInterface::__addResourceSpaceCloudItem(Name);
    }
}

void CQTInterface::onActionSave()
{
    const auto& FilePath = QFileDialog::getSaveFileName(this, tr("Save PointCloud"), ".", tr("PLY files(*.ply);;""PCD files(*.pcd)")).toStdString();

    PointCloud_t::Ptr pCloud2Save(new PointCloud_t);
    PointCloudRetouch::hiveDumpPointCloudtoSave(pCloud2Save);
	
    std::vector<PointCloud_t::Ptr> UserCloudSet;
    Visualization::hiveDumpUserCloudSet(UserCloudSet);

    for (auto pCloud : UserCloudSet)
        pCloud2Save->insert(pCloud2Save->end(), pCloud->begin(), pCloud->end());

    if (hiveObliquePhotography::hiveSavePointCloudScene(pCloud2Save, FilePath))
        __messageDockWidgetOutputText("Save scene successfully");
    else
        __messageDockWidgetOutputText("Scene is not saved");
}

void CQTInterface::onActionRubber()
{
    if (m_pVisualizationConfig)
        m_pVisualizationConfig->overwriteAttribute("RUBBER_MODE", m_UI.actionRubber->isChecked());
}

void CQTInterface::onActionBrush()
{
    
}

void CQTInterface::onActionOutlierDetection()
{
    if (!m_TileSet.TileSet.empty())
    {
        PointCloudRetouch::hiveMarkIsolatedAreaAsLitter();
        std::vector<std::size_t> PointLabelSet;
        PointCloudRetouch::hiveDumpPointLabel(PointLabelSet);
        Visualization::hiveRefreshVisualizer(PointLabelSet);
    }
}

void CQTInterface::onActionStartRepairHole()
{
    if (!m_TileSet.TileSet.empty())
    {
        PointCloud_t::Ptr pCloud(new PointCloud_t);
        PointCloudRetouch::hiveDumpPointCloudtoSave(pCloud);
        auto CloudSavedPath = "Temp/" + m_CurrentCloud + ".ply";
        hiveSavePointCloudScene(pCloud, CloudSavedPath);
        
        m_TileSet.TileSet = hiveInitPointCloudScene({ CloudSavedPath });
        PointCloudRetouch::hiveInit(m_TileSet.TileSet, m_pPointCloudRetouchConfig);
        Visualization::hiveInitVisualizer(m_TileSet.TileSet, true);
        __initialVTKWidget();

        //unable ui icons and reset configs
        {
            m_UI.actionPointPicking->setEnabled(false);
            m_UI.actionSave->setEnabled(true);
            m_UI.actionUpdate->setEnabled(false);
            m_UI.actionDeleteLitter->setEnabled(false);
            m_UI.actionDeleteBackground->setEnabled(false);
            m_UI.actionPrecompute->setEnabled(false);
            m_UI.actionRubber->setEnabled(false);
            m_UI.actionBrush->setEnabled(false);
            m_UI.actionSetting->setEnabled(true);
            m_UI.actionOutlierDetection->setEnabled(false);
            m_UI.actionRepairHole->setEnabled(true);

            m_pVisualizationConfig->overwriteAttribute(Visualization::AREA_MODE, false);
            m_pVisualizationConfig->overwriteAttribute(Visualization::CIRCLE_MODE, false);
            m_pVisualizationConfig->overwriteAttribute(Visualization::REPAIR_MODE, true);

            if (m_pPointPickingDockWidget)
                m_pPointPickingDockWidget->close();
            if (m_pAreaPickingSetting)
                _SAFE_DELETE(m_pAreaPickingSetting);
        }

        std::vector<std::size_t> PointLabelSet;
        PointCloudRetouch::hiveDumpPointLabel(PointLabelSet);
        Visualization::hiveRefreshVisualizer(PointLabelSet, true);

        __messageDockWidgetOutputText("Start hole repair.");
    }
}

void CQTInterface::onActionDiscardAndRecover()
{
    static int i = 1;
    if (i++ % 2)
        PointCloudRetouch::hiveHideLitter();
    else
        PointCloudRetouch::hiveDisplayLitter();

    std::vector<std::size_t> PointLabelSet;
    PointCloudRetouch::hiveDumpPointLabel(PointLabelSet);
    Visualization::hiveRefreshVisualizer(PointLabelSet);
}

void CQTInterface::onActionDeleteLitter()
{
    PointCloudRetouch::hiveRecoverLitterMark();
    Visualization::hiveRemoveAllShapes();
    Visualization::hiveCancelAllHighlighting();
    std::vector<std::size_t> PointLabelSet;
    PointCloudRetouch::hiveDumpPointLabel(PointLabelSet);
    Visualization::hiveRefreshVisualizer(PointLabelSet);
}

void CQTInterface::onActionDeleteBackground()
{
    PointCloudRetouch::hiveRecoverBackgroundMark();
    Visualization::hiveRemoveAllShapes();
    Visualization::hiveCancelAllHighlighting();
    std::vector<std::size_t> PointLabelSet;
    PointCloudRetouch::hiveDumpPointLabel(PointLabelSet);
    Visualization::hiveRefreshVisualizer(PointLabelSet);
}

void CQTInterface::onActionPrecompute()
{
    PointCloudRetouch::hiveRunPrecompute(m_CurrentCloud);
    m_UI.actionPrecompute->setChecked(false);
    __messageDockWidgetOutputText(m_CurrentCloud + "'s precompute is finished.");
}

void CQTInterface::onActionInstructions()
{
    m_pInstructionsDialog = new CInstructionsDialog(this);
    m_pInstructionsDialog->init();
    m_pInstructionsDialog->exec();
}

void CQTInterface::onActionSetting()
{
    if (m_pDisplayOptionsSettingDialog == nullptr)
    {
        m_pDisplayOptionsSettingDialog = new CDisplayOptionsSettingDialog(this, m_pDisplayOptionsSettingDialog);
        m_pDisplayOptionsSettingDialog->show();
        m_pDisplayOptionsSettingDialog->exec();

        __parseConfigFile();
        PointCloudRetouch::hiveInit(m_TileSet.TileSet, m_pPointCloudRetouchConfig);

        Visualization::hiveRemoveAllShapes();
        Visualization::hiveCancelAllHighlighting();

        std::vector<std::size_t> PointLabelSet;
        PointCloudRetouch::hiveDumpPointLabel(PointLabelSet);
        Visualization::hiveRefreshVisualizer(PointLabelSet);
    }
    else
        _SAFE_DELETE(m_pDisplayOptionsSettingDialog);
}

void CQTInterface::onActionReconstruction()
{
    auto MeshPath = QFileDialog::getSaveFileName(this, tr("Save Mesh"), m_MeshOpenPath.c_str(), tr("OBJ files(*.obj)")).toStdString();

    if (MeshPath == "")
        return;

    if (!m_TileSet.TileSet.empty())
    {
        m_MeshOpenPath = __getDirectory(MeshPath);

        PointCloud_t::Ptr pResult(new PointCloud_t);
        for (auto pCloud : m_TileSet.TileSet)
            *pResult += *pCloud;
        CMesh Mesh;
        SceneReconstruction::hiveSurfaceReconstruction(pResult, Mesh);
        m_MeshSet.NameSet.push_back(__getFileNameWithSuffix(MeshPath));
        m_MeshSet.MeshSet.push_back(Mesh);
        hiveSaveMeshModel(Mesh, MeshPath);

        __messageDockWidgetOutputText("Reconstruction finished.");
    }
}

void CQTInterface::onActionOpenMesh()
{
    QStringList FilePathList = QFileDialog::getOpenFileNames(this, tr("Open Mesh"), m_MeshOpenPath.c_str(), tr("OBJ files(*.obj)"));
    if (FilePathList.empty()) return;

    std::vector<std::string> FilePathSet;
    foreach(QString FilePathQString, FilePathList)
        FilePathSet.push_back(FilePathQString.toStdString());

    if (m_TileSet.TileSet.empty())
    {
        PointCloud_t::Ptr Temp(new PointCloud_t);
        Temp->push_back({});
        PointCloudRetouch::hiveInit({ Temp }, m_pPointCloudRetouchConfig);
        Visualization::hiveInitVisualizer({ Temp }, true);
        __initialVTKWidget();
    }

    //enable ui icons
    {
        m_UI.actionViewFromNegativeX->setEnabled(true);
        m_UI.actionViewFromPositiveX->setEnabled(true);
        m_UI.actionViewFromNegativeY->setEnabled(true);
        m_UI.actionViewFromPositiveY->setEnabled(true);
        m_UI.actionViewFromNegativeZ->setEnabled(true);
        m_UI.actionViewFromPositiveZ->setEnabled(true);

        m_pVisualizationConfig->overwriteAttribute(Visualization::REPAIR_MODE, false);
    }

    m_MeshOpenPath = __getDirectory(FilePathSet.front());
    for (const auto& FilePath : FilePathSet)
    {
        if (FilePath.empty()) continue;

        CMesh Mesh;
        auto MeshName = __getFileNameWithSuffix(FilePath);
        hiveLoadMeshModel(Mesh, FilePath); //1
        m_MeshSet.NameSet.push_back(MeshName);
        m_MeshSet.MeshSet.push_back(Mesh);

        Visualization::hiveAddTextureMesh(Mesh.toTexMesh({})); //2
        __addResourceSpaceMeshItem(MeshName);
        __messageDockWidgetOutputText("Open mesh " + FilePath + " succeed.");
    }
    Visualization::hiveSetVisualFlag(Visualization::EVisualFlag::ShowMesh);
    std::vector<std::size_t> PointLabelSet;
    PointCloudRetouch::hiveDumpPointLabel(PointLabelSet);
    Visualization::hiveRefreshVisualizer(PointLabelSet, true);
}

void CQTInterface::onActionSutureMesh()
{
    if (m_SelectedMeshIndices.size() == 2)
    {
        auto MeshIter = m_SelectedMeshIndices.begin();
        auto MeshOneIndex = *MeshIter++;
        auto MeshTwoIndex = *MeshIter;
        auto& MeshOne = m_MeshSet.MeshSet[MeshOneIndex];
        auto& MeshTwo = m_MeshSet.MeshSet[MeshTwoIndex];

        SceneReconstruction::hiveSutureMesh(MeshOne, MeshTwo);

        hiveSaveMeshModel(MeshOne, m_MeshSet.NameSet[MeshOneIndex]);
        hiveSaveMeshModel(MeshTwo, m_MeshSet.NameSet[MeshTwoIndex]);

        __messageDockWidgetOutputText("Suture mesh " + m_MeshSet.NameSet[MeshOneIndex] + " and " + m_MeshSet.NameSet[MeshTwoIndex] + " succeed");
    }
    else
        __messageDockWidgetOutputText("Selected num mesh and cloud is not two, num mesh: " + std::to_string(m_SelectedMeshIndices.size()) + " num cloud: " + std::to_string(m_SelectedTileIndices.size()));
}

void CQTInterface::onActionShowMeshIndices()
{
    if (m_SelectedMeshIndices.size() == 1)
    {
        auto IndicesPath = QFileDialog::getOpenFileName(this, tr("Open Indices"), m_MeshOpenPath.c_str(), tr("Indices files(*.txt)")).toStdString();

        Visualization::TestInterface(m_MeshSet.MeshSet[*m_SelectedMeshIndices.begin()], IndicesPath);
    }
}

void CQTInterface::onActionParameterization()
{
    auto MeshPath = QFileDialog::getSaveFileName(this, tr("Save Mesh"), m_MeshOpenPath.c_str(), tr("OBJ files(*.obj)")).toStdString();
    auto Directory = __getDirectory(MeshPath);

    if (!m_SelectedMeshIndices.empty() && MeshPath != "")
    {
        for (auto Index : m_SelectedMeshIndices)
        {
            if (SceneReconstruction::hiveMeshParameterization(m_MeshSet.MeshSet[Index]))
            {
                hiveSaveMeshModel(m_MeshSet.MeshSet[Index], Directory + "/" + m_MeshSet.NameSet[Index]);
                __messageDockWidgetOutputText("Mesh parameterization succeed.");
            }
            else
            {
                __messageDockWidgetOutputText("Mesh parameterization failed! Check the log for more information.");
            }
        }

    }
}

void CQTInterface::onActionBakeTexture()
{
    auto TexturePath = QFileDialog::getSaveFileName(this, tr("Save Texture"), ".", tr("PNG files(*.png)")).toStdString();

    if (!m_SelectedTileIndices.empty() && m_SelectedMeshIndices.size() == 1)
    {
        PointCloud_t::Ptr pResult(new PointCloud_t);
        for (auto Index : m_SelectedTileIndices)
            *pResult += *m_TileSet.TileSet[Index];
        CImage<std::array<int, 3>> Texture;
        
        if (SceneReconstruction::hiveBakeColorTexture(m_MeshSet.MeshSet[*m_SelectedMeshIndices.begin()], pResult, Texture))
        {
            const auto Width = Texture.getWidth();
            const auto Height = Texture.getHeight();
            const auto BytesPerPixel = 3;
            auto ResultImage = new unsigned char[Width * Height * BytesPerPixel];
            for (auto i = 0; i < Height; i++)
                for (auto k = 0; k < Width; k++)
                {
                    auto Offset = ((Height - 1 - i) * Width + k) * BytesPerPixel;
                    ResultImage[Offset] = Texture.getColor(i, k)[0];
                    ResultImage[Offset + 1] = Texture.getColor(i, k)[1];
                    ResultImage[Offset + 2] = Texture.getColor(i, k)[2];
                }

            stbi_write_png(TexturePath.c_str(), Width, Height, BytesPerPixel, ResultImage, 0);
            stbi_image_free(ResultImage);
            __messageDockWidgetOutputText("Bake Texture finished.");
        }
        else
            __messageDockWidgetOutputText("Bake Texture failed.");      
    }
}

void CQTInterface::onActionReconstructionStitching()
{
    if (m_TileSet.TileSet.size() != 2)
    {
        __messageDockWidgetOutputText("Please load 2 Tile.");
        return;
    }

    std::vector<std::string> MeshPaths;
    int MeshIndex;
    for (int i = 0; i < m_TileSet.TileSet.size(); i++)
        MeshPaths.push_back(QFileDialog::getSaveFileName(this, tr("SaveMesh"), m_MeshOpenPath.c_str(), tr("OBJ files(*.obj)")).toStdString());
        
    for (int i = 0; i < m_TileSet.TileSet.size(); i++)
    {
        CMesh Mesh;
        SceneReconstruction::hiveSurfaceReconstruction(m_TileSet.TileSet[i], Mesh);
        m_MeshSet.NameSet.push_back(__getFileNameWithSuffix(MeshPaths[i]));
        m_MeshSet.MeshSet.push_back(Mesh);
        MeshIndex = m_MeshSet.MeshSet.size() - 1;
        //hiveSaveMeshModel(Mesh, MeshPaths[i]);
    }

    __messageDockWidgetOutputText("Reconstruction finished.");

    auto& MeshOne = m_MeshSet.MeshSet[MeshIndex - 1];
    auto& MeshTwo = m_MeshSet.MeshSet[MeshIndex];

    SceneReconstruction::hiveSutureMesh(MeshOne, MeshTwo);

    hiveSaveMeshModel(MeshOne, MeshPaths[MeshIndex - 1]);
    hiveSaveMeshModel(MeshTwo, MeshPaths[MeshIndex]);

    __messageDockWidgetOutputText("Suture mesh " + m_MeshSet.NameSet[MeshIndex - 1] + " and " + m_MeshSet.NameSet[MeshIndex] + " succeed");
}

void CQTInterface::onActionAutoModeling()
{
    QStringList FilePathList = QFileDialog::getOpenFileNames(this, tr("Open PointCloud or Mesh"), QString::fromStdString(m_CloudOpenPath), tr("PointCloud Files(*.pcd *.ply);;"));
    if (FilePathList.empty())
        return;

    foreach(QString FilePathQString, FilePathList)
    {
        std::vector<std::string> FilePath = { FilePathQString.toStdString() };
        std::vector<PointCloud_t::Ptr> Tile = hiveInitPointCloudScene(FilePath);
        m_CloudOpenPath = __getDirectory(FilePath.back());
        if (!Tile.empty())
        {
            SceneReconstruction::hiveRecordTileInfo(Tile.back(), __getFileName(FilePath.back()));

            CMesh Mesh;
            SceneReconstruction::hiveSurfaceReconstruction(Tile.back(), Mesh);
            std::string OutputMeshPath = __getDirectory(FilePath.back()) + std::string("/Output/") + __getFileName(FilePath.back()) + std::string(".obj");
            hiveSaveMeshModel(Mesh, OutputMeshPath);
            __messageDockWidgetOutputText("Reconstruction of " + __getFileNameWithSuffix(FilePath.back()) + std::string(" is finished."));

            //TODO: delete ����

            FilePath.clear();
            Tile.clear();
        }
    }

    __autoSutureMeshes(FilePathList);

    //TODO: ����決
}

// TODO::copy-paste
void CQTInterface::onActionParameterizationBake()
{
    if (m_SelectedTileIndices.size() != 1)
    {
        __messageDockWidgetOutputText("Please load one Tile.");
        return;
    }

    if (m_SelectedMeshIndices.size() != 1)
    {
        __messageDockWidgetOutputText("No Tile Loaded.");
        return;
    }

    std::vector<std::string> MeshPaths;
    std::vector<std::string> TexturePaths;
    int MeshIndex;
    for (int i = 0; i < m_SelectedTileIndices.size(); i++)
    {
        auto MeshPath = QFileDialog::getSaveFileName(this, tr("Save Mesh"), m_MeshOpenPath.c_str(), tr("OBJ files(*.obj)")).toStdString();
        if (MeshPath == "")
            break;
        MeshPaths.push_back(MeshPath);
        auto Directory = __getDirectory(MeshPath);
        auto FileName = __getFileName(MeshPath);
        TexturePaths.push_back(Directory + "/" +  FileName + ".png");
    }

    if (MeshPaths.size() != m_SelectedMeshIndices.size())
    {
        __messageDockWidgetOutputText("Please input Path.");
        return;
    }

    for (auto Index : m_SelectedMeshIndices)
    {
        if (SceneReconstruction::hiveMeshParameterization(m_MeshSet.MeshSet[Index]))
        {
            hiveSaveMeshModel(m_MeshSet.MeshSet[Index], MeshPaths[0]);
            __messageDockWidgetOutputText("Mesh parameterization succeed.");
        }
        else
            __messageDockWidgetOutputText("Mesh parameterization failed! Check the log for more information.");
    }

    PointCloud_t::Ptr pResult(new PointCloud_t);
    for (auto Index : m_SelectedTileIndices)
        *pResult += *m_TileSet.TileSet[Index];
    CImage<std::array<int, 3>> Texture;

    if (SceneReconstruction::hiveBakeColorTexture(m_MeshSet.MeshSet[*m_SelectedMeshIndices.begin()], pResult, Texture))
    {
        const auto Width = Texture.getWidth();
        const auto Height = Texture.getHeight();
        const auto BytesPerPixel = 3;
        auto ResultImage = new unsigned char[Width * Height * BytesPerPixel];
        for (auto i = 0; i < Height; i++)
            for (auto k = 0; k < Width; k++)
            {
                auto Offset = ((Height - 1 - i) * Width + k) * BytesPerPixel;
                ResultImage[Offset] = Texture.getColor(i, k)[0];
                ResultImage[Offset + 1] = Texture.getColor(i, k)[1];
                ResultImage[Offset + 2] = Texture.getColor(i, k)[2];
            }

        stbi_write_png(TexturePaths[0].c_str(), Width, Height, BytesPerPixel, ResultImage, 0);
        stbi_image_free(ResultImage);
        __messageDockWidgetOutputText("Bake Texture finished.");
    }
    else
        __messageDockWidgetOutputText("Bake Texture failed.");
}

void CQTInterface::onResourceSpaceItemDoubleClick(QModelIndex)
{
    if (!m_TileSet.TileSet.empty())
    {
        Visualization::hiveResetVisualizer(m_TileSet.TileSet, true);
        __initialVTKWidget();
        std::vector<std::size_t> PointLabelSet;
        PointCloudRetouch::hiveDumpPointLabel(PointLabelSet);
        Visualization::hiveRefreshVisualizer(PointLabelSet);
    }
}

void CQTInterface::onResourceSpaceItemClick(QModelIndex vIndex)
{
    //auto ModelName = m_pResourceSpaceStandardItemModels->item(vIndex.row())->text().toStdString();

    //auto processSelect = [&](const std::vector<std::string>& vNameSet, std::set<int>& vioIndexSet, const std::string& vModelName)
    //{
    //    auto NameIter = std::find(vNameSet.begin(), vNameSet.end(), ModelName);
    //    if (NameIter != vNameSet.end())
    //    {
    //        auto Index = std::distance(vNameSet.begin(), NameIter);
    //        auto IndexIter = std::find(vioIndexSet.begin(), vioIndexSet.end(), Index);
    //        if (IndexIter == vioIndexSet.end())
    //            vioIndexSet.insert(Index);
    //        else
    //            vioIndexSet.erase(Index); // ��ѡ
    //    }

    //    std::string Text;
    //    Text += "Current select " + std::to_string(vioIndexSet.size()) + " " + vModelName + "s, which are \n";
    //    for (auto Index : vioIndexSet)
    //        Text += "[" + vNameSet[Index] + "] ";
    //    Text += "\n";
    //    __messageDockWidgetOutputText(Text);
    //};

    //if (__getSuffix(ModelName) == "ply" || __getSuffix(ModelName) == "pcd")
    //    processSelect(m_TileSet.NameSet, m_SelectedTileIndices, "tile");
    //else if (__getSuffix(ModelName) == "obj")
    //    processSelect(m_MeshSet.NameSet, m_SelectedMeshIndices, "mesh");
}

void CQTInterface::onResourceSpaceItemChange(QStandardItem* vItem)
{
    auto ModelName = vItem->text().toStdString();

    auto processSelect = [&](const std::vector<std::string>& vNameSet, std::set<int>& vioIndexSet, const std::string& vModelName)
    {
        auto NameIter = std::find(vNameSet.begin(), vNameSet.end(), ModelName);
        if (NameIter != vNameSet.end())
        {
            auto Index = std::distance(vNameSet.begin(), NameIter);
            if (vItem->checkState() == Qt::CheckState::Checked)
                vioIndexSet.insert(Index);
            else if (vItem->checkState() == Qt::CheckState::Unchecked)
                vioIndexSet.erase(Index);
        }

        std::string Text;
        Text += "Current select " + std::to_string(vioIndexSet.size()) + " " + vModelName + "s, which are \n";
        for (auto Index : vioIndexSet)
            Text += "[" + vNameSet[Index] + "] ";
        Text += "\n";
        __messageDockWidgetOutputText(Text);
    };

    if (__getSuffix(ModelName) == "ply" || __getSuffix(ModelName) == "pcd")
        processSelect(m_TileSet.NameSet, m_SelectedTileIndices, "tile");
    else if (__getSuffix(ModelName) == "obj")
        processSelect(m_MeshSet.NameSet, m_SelectedMeshIndices, "mesh");
}

void CQTInterface::onChangeCameraLookStraightAxis()
{
    QAction* pAction = qobject_cast<QAction*>(sender());

    std::unordered_map<std::string, size_t> ActionNameMap = {
        {"actionViewFromPositiveX",1},{"actionViewFromNegativeX",2},
        {"actionViewFromPositiveY",3},{"actionViewFromNegativeY",4},
        {"actionViewFromPositiveZ",5},{"actionViewFromNegativeZ",6} };

    hiveObliquePhotography::Visualization::hiveChangeCameraLookStraightAxis(ActionNameMap.at(pAction->objectName().toStdString()));
}

void CQTInterface::onChangeLanguage()
{
    QAction* pAction = qobject_cast<QAction*>(sender());
    std::unordered_map<std::string, std::string> ActionNameMap = {
        {"actionChinese","zh"},{"actionEnglish","en"}, };

    QTranslator* pTranslator = new QTranslator;
    if (ActionNameMap.at(pAction->objectName().toStdString()) == "zh")
    {
        if (pTranslator->load("qtinterface_en.qm"))
        {
            qApp->installTranslator(pTranslator);
        }
    }
    else if (ActionNameMap.at(pAction->objectName().toStdString()) == "en")
    {
        if (pTranslator->load("qtinterface_zh.qm"))
        {
            qApp->installTranslator(pTranslator);
        }
    }

    m_UI.retranslateUi(this);
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
    QDialog* pExitDialog = new QDialog(this);
    pExitDialog->setWindowFlag(Qt::FramelessWindowHint);
    pExitDialog->deleteLater();
    pExitDialog->resize(200, 100);
    QLabel* pLabel = new QLabel(pExitDialog);
    pLabel->setText("Are you sure?");

    pLabel->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
    QPushButton* pYesButton = new QPushButton(pExitDialog);
    QPushButton* pNoButton = new QPushButton(pExitDialog);
    pYesButton->setStyleSheet("QPushButton{color: black;min-width:75px;max-width:75px;min-height:20px;border:1px solid white;border-radius:5px;}"
        "QPushButton:hover{background-color: #2292DD;border-color: #000000;color:rgb(255, 255, 255);}"
        "QPushButton:pressed{background-color: #111111;border-color: #333333;color: blue;}");
    pNoButton->setStyleSheet("QPushButton{color: black;min-width:75px;max-width:75px;min-height:20px;border:1px solid white;border-radius:5px;}"
        "QPushButton:hover{background-color: #2292DD;border-color: #000000;color:rgb(255, 255, 255);}"
        "QPushButton:pressed{background-color: #111111;border-color: #333333;color: blue;}");
    pYesButton->setText("Yes");
    pNoButton->setText("No");
    pYesButton->setMaximumWidth(100);
    pNoButton->setMaximumWidth(100);

    QObject::connect(pYesButton, &QPushButton::clicked, [=]()
        {
            pExitDialog->done(1);
        });
    QObject::connect(pNoButton, &QPushButton::clicked, [=]()
        {
            pExitDialog->done(0);
        });

    QHBoxLayout* pHBoxLayout = new QHBoxLayout();
    pHBoxLayout->setSpacing(5);
    pHBoxLayout->addStretch();
    pHBoxLayout->addWidget(pYesButton);
    pHBoxLayout->addWidget(pNoButton);
    QVBoxLayout* pVBoxLayout = new QVBoxLayout();
    pVBoxLayout->addWidget(pLabel);
    pVBoxLayout->addItem(pHBoxLayout);
    pExitDialog->setLayout(pVBoxLayout);
    if (pExitDialog->exec() == 1)
    {
        vEvent->accept();
    }
    else
    {
        vEvent->ignore();
    }
}

void CQTInterface::__addAxesToView(vtkRenderWindowInteractor* vInteractor, double vX, double vY, double vXWide, double vYWide)
{
    vtkSmartPointer<vtkAxesActor> pAxes = vtkSmartPointer<vtkAxesActor>::New();

    m_pAxes = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    m_pAxes->SetOutlineColor(0.9300, 0.5700, 0.1300);
    m_pAxes->SetOrientationMarker(pAxes);
    m_pAxes->SetInteractor(vInteractor);
    m_pAxes->SetViewport(vX, vY, vXWide, vYWide);
    m_pAxes->SetEnabled(true);
    m_pAxes->InteractiveOn();
}