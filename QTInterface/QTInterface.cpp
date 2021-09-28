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
    QObject::connect(m_UI.actionParameterization, SIGNAL(triggered()), this, SLOT(onActionParameterization()));
    QObject::connect(m_UI.actionBakeTexture, SIGNAL(triggered()), this, SLOT(onActionBakeTexture()));
    QObject::connect(m_UI.resourceSpaceTreeView, SIGNAL(doubleClicked(QModelIndex)), this, SLOT(onResourceSpaceItemDoubleClick(QModelIndex)));
    QObject::connect(m_UI.actionInstructions, SIGNAL(triggered()), this, SLOT(onActionInstructions()));
    QObject::connect(m_UI.actionOutlierDetection, SIGNAL(triggered()), this, SLOT(onActionOutlierDetection()));
    QObject::connect(m_UI.actionRepairHole, SIGNAL(triggered()), this, SLOT(onActionStartRepairHole()));
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

bool CQTInterface::__addResourceSpaceCloudItem(const std::string& vFilePath)
{
    const auto& FileName = CQTInterface::__getFileName(vFilePath);

    QStandardItem* pStandardItem = new QStandardItem(QString::fromStdString(FileName));
    pStandardItem->setCheckable(true);
    pStandardItem->setCheckState(Qt::Checked);
    pStandardItem->setEditable(false);
    m_pResourceSpaceStandardItemModels->removeRow(0);
    m_pResourceSpaceStandardItemModels->appendRow(pStandardItem);

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
    if (!m_TileSet.empty())
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
            CQTInterface::__messageDockWidgetOutputText(QString::fromStdString("Switch to point picking mode."));
        }
        else
        {
            m_pPointPickingDockWidget->close();

            std::vector<std::size_t> PointLabelSet;
            PointCloudRetouch::hiveDumpPointLabel(PointLabelSet);
            Visualization::hiveRefreshVisualizer(PointLabelSet);
            CQTInterface::__messageDockWidgetOutputText(QString::fromStdString("Switch to view mode."));
        }

        if (m_pVisualizationConfig)
            m_pVisualizationConfig->overwriteAttribute(Visualization::CIRCLE_MODE, m_UI.actionPointPicking->isChecked());
    }

}

void CQTInterface::onActionAreaPicking()
{
    if (!m_TileSet.empty())
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
                CQTInterface::__messageDockWidgetOutputText(QString::fromStdString("Switch to area picking mode."));

            }
            else
            {
                if (m_pAreaPickingSetting)
                    _SAFE_DELETE(m_pAreaPickingSetting);
                CQTInterface::__messageDockWidgetOutputText(QString::fromStdString("Switch to view mode."));
            }
        }

        m_pVisualizationConfig->overwriteAttribute(Visualization::AREA_MODE, m_UI.actionAreaPicking->isChecked());
    }
}

void CQTInterface::onActionOpen()
{
    QStringList FilePathList = QFileDialog::getOpenFileNames(this, tr("Open PointCloud or Mesh"), QString::fromStdString(m_DirectoryOpenPath), tr("PointCloud Files(*.pcd *.ply);;" "OBJ files(*.obj)"));
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

    m_TileSet = hiveInitPointCloudScene(FilePathSet);

    if (m_TileSet.empty())
        FileOpenSuccessFlag = false;

    if (FileOpenSuccessFlag)
    {
        m_DirectoryOpenPath = __getDirectory(FilePathSet.back());

        auto config = m_pPointCloudRetouchConfig->getSubconfigAt(0);
        auto num = config->getNumSubconfig();

        PointCloudRetouch::hiveInit(m_TileSet, m_pPointCloudRetouchConfig);
        Visualization::hiveInitVisualizer(m_TileSet, true);
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

            m_pVisualizationConfig->overwriteAttribute(Visualization::REPAIR_MODE, false);
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
    const auto& FilePath = QFileDialog::getSaveFileName(this, tr("Save PointCloud"), ".", tr("PLY files(*.ply);;""PCD files(*.pcd)")).toStdString();

    PointCloud_t::Ptr pCloud2Save(new PointCloud_t);
    PointCloudRetouch::hiveDumpPointCloudtoSave(pCloud2Save);
    if (m_pVisualizationConfig->getAttribute<bool>(Visualization::REPAIR_MODE).value())
    {
        std::vector<PointCloud_t::Ptr> UserCloudSet;
        Visualization::hiveDumpUserCloudSet(UserCloudSet);

        for (auto pCloud : UserCloudSet)
            pCloud2Save->insert(pCloud2Save->end(), pCloud->begin(), pCloud->end());
    }

    if (hiveObliquePhotography::hiveSavePointCloudScene(*pCloud2Save, FilePath))
        __messageDockWidgetOutputText(QString::fromStdString("Save scene successfully"));
    else
        __messageDockWidgetOutputText(QString::fromStdString("Scene is not saved"));

    /*CMesh Mesh;
    SceneReconstruction::hiveSurfaceReconstruction(m_pCloud, Mesh);
    pcl::io::savePLYFileBinary("Temp/TestPoisson.ply", Mesh.toPolMesh());*/
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
    if (!m_TileSet.empty())
    {
        PointCloudRetouch::hiveMarkIsolatedAreaAsLitter();
        std::vector<std::size_t> PointLabelSet;
        PointCloudRetouch::hiveDumpPointLabel(PointLabelSet);
        Visualization::hiveRefreshVisualizer(PointLabelSet);
    }
}

void CQTInterface::onActionStartRepairHole()
{
    if (!m_TileSet.empty())
    {
        PointCloud_t::Ptr pCloud(new PointCloud_t);
        PointCloudRetouch::hiveDumpPointCloudtoSave(pCloud);
        auto CloudSavedPath = "Temp/" + m_CurrentCloud + ".ply";
        hiveSavePointCloudScene(*pCloud, CloudSavedPath);
        
        m_TileSet = hiveInitPointCloudScene({ CloudSavedPath });
        PointCloudRetouch::hiveInit(m_TileSet, m_pPointCloudRetouchConfig);
        Visualization::hiveInitVisualizer(m_TileSet, true);
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

        __messageDockWidgetOutputText(QString::fromStdString("Start hole repair."));
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
    __messageDockWidgetOutputText(QString::fromStdString(m_CurrentCloud + "'s precompute is finished."));
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
        PointCloudRetouch::hiveInit(m_TileSet, m_pPointCloudRetouchConfig);

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
    auto MeshPath = QFileDialog::getSaveFileName(this, tr("Save Mesh"), ".", tr("OBJ files(*.obj)")).toStdString();

    if (!m_TileSet.empty())
    {
        PointCloud_t::Ptr pResult(new PointCloud_t);
        for (auto pCloud : m_TileSet)
            *pResult += *pCloud;
        CMesh Mesh;
        SceneReconstruction::hiveSurfaceReconstruction(pResult, Mesh);
        m_MeshSet[MeshPath] = Mesh;
        hiveSaveMeshModel(Mesh, MeshPath);

        __messageDockWidgetOutputText(QString::fromStdString("Reconstruction finished."));
    }
}

void CQTInterface::onActionOpenMesh()
{
    auto MeshPath = QFileDialog::getOpenFileName(this, tr("Open Mesh"), QString::fromStdString("../UnitTests/Unit_Test_026"), tr("OBJ files(*.obj)")).toStdString();

    // load mesh
    if (MeshPath != "" && hiveUtility::hiveGetFileSuffix(MeshPath) == "obj")
    {
        m_MeshSet[MeshPath] = SceneReconstruction::hiveTestCMesh(MeshPath);
        Visualization::hiveSetVisualFlag(Visualization::EVisualFlag::ShowMesh);
        Visualization::TestInterface(MeshPath, "../UnitTests/Unit_Test_026/BoundaryPoints.txt");
        __messageDockWidgetOutputText(QString::fromStdString("Open mesh succeed."));
    }
}

void CQTInterface::onActionParameterization()
{
    std::string PresetPath(".");
    if (!m_MeshSet.empty())
    {
        PresetPath = m_MeshSet.begin()->first;
        PresetPath = PresetPath.substr(0, PresetPath.find_last_of("/"));
    }
    auto MeshPath = QFileDialog::getSaveFileName(this, tr("Save Mesh"), PresetPath.c_str(), tr("OBJ files(*.obj)")).toStdString();

    if (!m_MeshSet.empty() && MeshPath != "")
    {
        SceneReconstruction::hiveMeshParameterization(m_MeshSet.begin()->second, m_MeshSet.begin()->first);
        hiveSaveMeshModel(m_MeshSet.begin()->second, MeshPath);
        __messageDockWidgetOutputText(QString::fromStdString("Mesh parameterization succeed."));
    }
}

void CQTInterface::onActionBakeTexture()
{
    auto TexturePath = QFileDialog::getSaveFileName(this, tr("Save Texture"), ".", tr("PNG files(*.png)")).toStdString();

    if (!m_TileSet.empty())
    {
        PointCloud_t::Ptr pResult(new PointCloud_t);
        for (auto pCloud : m_TileSet)
            *pResult += *pCloud;
        auto Texture = SceneReconstruction::hiveBakeColorTexture(m_MeshSet.begin()->second, pResult, { 2048, 2048 });
        
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
        }

        __messageDockWidgetOutputText(QString::fromStdString("Bake Texture finished."));
    }
}

void CQTInterface::onResourceSpaceItemDoubleClick(QModelIndex)
{
    Visualization::hiveResetVisualizer(m_TileSet, true);
    __initialVTKWidget();
    std::vector<std::size_t> PointLabelSet;
    PointCloudRetouch::hiveDumpPointLabel(PointLabelSet);
    Visualization::hiveRefreshVisualizer(PointLabelSet);
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