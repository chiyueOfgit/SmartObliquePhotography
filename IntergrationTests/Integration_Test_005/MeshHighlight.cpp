#include "pch.h"
#include "MeshHighlight.h"
#include <QtWidgets/qmdisubwindow.h>
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

#include "ObliquePhotographyDataInterface.h"
#include "PointCloudRetouchInterface.h"
#include "SceneReconstructionInterface.h"
#include "VisualizationInterface.h"
#include "PointCloudRetouchConfig.h"
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

using namespace hiveObliquePhotography::MeshHighlight;

CMeshHighlight::CMeshHighlight(QWidget * vParent)
    : QMainWindow(vParent)
{
    m_UI.setupUi(this);
}

CMeshHighlight::~CMeshHighlight()
{
}

void CMeshHighlight::init()
{
    {
        Visualization::hiveGetVisualizationConfig(m_pVisualizationConfig);
    }
    __connectSignals();
    __parseConfigFile();
}

void CMeshHighlight::__connectSignals()
{
    QObject::connect(m_UI.actionSave, SIGNAL(triggered()), this, SLOT(onActionSave()));
    QObject::connect(m_UI.actionReconstruction, SIGNAL(triggered()), this, SLOT(onActionReconstruction()));
    QObject::connect(m_UI.actionOpenMesh, SIGNAL(triggered()), this, SLOT(onActionOpenMesh()));
    QObject::connect(m_UI.actionSutureMesh, SIGNAL(triggered()), this, SLOT(onActionSutureMesh()));
    QObject::connect(m_UI.actionShowMeshIndices, SIGNAL(triggered()), this, SLOT(onActionShowMeshIndices()));
    QObject::connect(m_UI.actionParameterization, SIGNAL(triggered()), this, SLOT(onActionParameterization()));
    QObject::connect(m_UI.actionBakeTexture, SIGNAL(triggered()), this, SLOT(onActionBakeTexture()));
    QObject::connect(m_pResourceSpaceStandardItemModels, SIGNAL(itemChanged(QStandardItem*)), this, SLOT(onResourceSpaceItemChange(QStandardItem*)));
    QObject::connect(m_UI.actionRepairHole, SIGNAL(triggered()), this, SLOT(onActionStartRepairHole()));
}

void CMeshHighlight::__initialVTKWidget()
{
    auto pViewer = static_cast<pcl::visualization::PCLVisualizer*>(Visualization::hiveGetPCLVisualizer());
    m_UI.VTKWidget->SetRenderWindow(pViewer->getRenderWindow());
    pViewer->setupInteractor(m_UI.VTKWidget->GetInteractor(), m_UI.VTKWidget->GetRenderWindow());
    __addAxesToView(pViewer->getRenderWindow()->GetInteractor(), 0.0, 0.0, 0.2, 0.2);
    m_UI.VTKWidget->update();
}

void CMeshHighlight::__parseConfigFile()
{
    const std::string ConfigPath = "PointCloudRetouchConfig.xml";
    m_pPointCloudRetouchConfig = new hiveObliquePhotography::PointCloudRetouch::CPointCloudRetouchConfig;
    if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, m_pPointCloudRetouchConfig) != hiveConfig::EParseResult::SUCCEED)
    {
        _HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
        return;
    }
}

std::string CMeshHighlight::__getFileName(const std::string& vFilePath)
{
    return vFilePath.substr(vFilePath.find_last_of('/') + 1, vFilePath.find_last_of('.') - vFilePath.find_last_of('/') - 1);
}

std::string CMeshHighlight::__getDirectory(const std::string& vFilePath)
{
    return vFilePath.substr(0, vFilePath.find_last_of('/'));
}

std::string CMeshHighlight::__getFileNameWithSuffix(const std::string& vFilePath)
{
    return vFilePath.substr(vFilePath.find_last_of('/') + 1, vFilePath.length());
}

std::string CMeshHighlight::__getSuffix(const std::string& vFilePath)
{
    return vFilePath.substr(vFilePath.find_last_of('.') + 1, vFilePath.length());
}

void CMeshHighlight::onActionSave()
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
}

void CMeshHighlight::onActionReconstruction()
{
    auto MeshPath = QFileDialog::getSaveFileName(this, tr("Save Mesh"), m_MeshOpenPath.c_str(), tr("OBJ files(*.obj)")).toStdString();

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
    }
}

void CMeshHighlight::onActionOpenMesh()
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

    m_MeshOpenPath = __getDirectory(FilePathSet.front());
    for (const auto& FilePath : FilePathSet)
    {
        if (FilePath.empty()) continue;

        CMesh Mesh;
        auto MeshName = __getFileNameWithSuffix(FilePath);
        hiveLoadMeshModel(Mesh, FilePath);
        m_MeshSet.NameSet.push_back(MeshName);
        m_MeshSet.MeshSet.push_back(Mesh);

        Visualization::hiveAddTextureMesh(Mesh.toTexMesh({}));
    }
    Visualization::hiveSetVisualFlag(Visualization::EVisualFlag::ShowMesh);
    std::vector<std::size_t> PointLabelSet;
    PointCloudRetouch::hiveDumpPointLabel(PointLabelSet);
    Visualization::hiveRefreshVisualizer(PointLabelSet, true);
}

void CMeshHighlight::onActionSutureMesh()
{
    if (m_SelectedMeshIndices.size() == 2 && m_SelectedTileIndices.size() == 2)
    {
        auto MeshIter = m_SelectedMeshIndices.begin();
        auto MeshOneIndex = *MeshIter++;
        auto MeshTwoIndex = *MeshIter;
        auto& MeshOne = m_MeshSet.MeshSet[MeshOneIndex];
        auto& MeshTwo = m_MeshSet.MeshSet[MeshTwoIndex];
        auto TileIter = m_SelectedTileIndices.begin();
        auto CloudOne = m_TileSet.TileSet[*TileIter++];
        auto CloudTwo = m_TileSet.TileSet[*TileIter];

        SceneReconstruction::hiveSutureMesh(MeshOne, MeshTwo, CloudOne, CloudTwo);

        hiveSaveMeshModel(MeshOne, m_MeshSet.NameSet[MeshOneIndex]);
        hiveSaveMeshModel(MeshTwo, m_MeshSet.NameSet[MeshTwoIndex]);
    }
}

void CMeshHighlight::onActionShowMeshIndices()
{
    if (m_SelectedMeshIndices.size() == 1)
    {
        auto IndicesPath = QFileDialog::getOpenFileName(this, tr("Open Indices"), m_MeshOpenPath.c_str(), tr("Indices files(*.txt)")).toStdString();

        Visualization::TestInterface(m_MeshSet.MeshSet[*m_SelectedMeshIndices.begin()], IndicesPath);
    }
}

void CMeshHighlight::onActionParameterization()
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
            }
        }

    }
}

void CMeshHighlight::onActionBakeTexture()
{
    auto TexturePath = QFileDialog::getSaveFileName(this, tr("Save Texture"), ".", tr("PNG files(*.png)")).toStdString();

    if (!m_SelectedTileIndices.empty() && m_SelectedMeshIndices.size() == 1)
    {
        PointCloud_t::Ptr pResult(new PointCloud_t);
        for (auto Index : m_SelectedTileIndices)
            *pResult += *m_TileSet.TileSet[Index];
        auto Texture = SceneReconstruction::hiveBakeColorTexture(m_MeshSet.MeshSet[*m_SelectedMeshIndices.begin()], pResult, { 512, 512 });
        
        if (Texture.getHeight() <= 0 || Texture.getWidth() <= 0)
        {
            return;
        }
        
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
    }
}

void CMeshHighlight::keyPressEvent(QKeyEvent* vEvent)
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

void CMeshHighlight::closeEvent(QCloseEvent* vEvent)
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

void CMeshHighlight::__addAxesToView(vtkRenderWindowInteractor* vInteractor, double vX, double vY, double vXWide, double vYWide)
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