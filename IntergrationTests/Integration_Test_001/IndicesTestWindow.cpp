#include "pch.h"
#include "IndicesTestWindow.h"
#include <QtWidgets/qmdisubwindow.h>
#include <QSlider>
#include <QtWidgets/QFileDialog>
#include <qobject.h>
#include <vtkRenderWindow.h>
#include <vtkAutoInit.h>

#include "ObliquePhotographyDataInterface.h"
#include "PointCloudRetouchInterface.h"
#include "VisualizationInterface.h"
#include "PointCloudRetouchConfig.h"

#include <fstream>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

using namespace hiveObliquePhotography;

CSingleStepWindow::CSingleStepWindow(QWidget * vParent)
    : QMainWindow(vParent)
{
    Visualization::hiveGetVisualizationConfig(m_pVisualizationConfig);

    m_WindowUI.setupUi(this);

    __connectSignals();
    __parseConfigFile();
}

CSingleStepWindow::~CSingleStepWindow()
{
}

void CSingleStepWindow::__connectSignals()
{
    QObject::connect(m_WindowUI.actionOpen, SIGNAL(triggered()), this, SLOT(__onActionOpen()));
    QObject::connect(m_WindowUI.actionLoad, SIGNAL(triggered()), this, SLOT(__onActionLoad()));
    QObject::connect(m_WindowUI.actionClear, SIGNAL(triggered()), this, SLOT(__onActionClear()));
}

void CSingleStepWindow::__initialVTKWidget()
{
    auto pViewer = static_cast<pcl::visualization::PCLVisualizer*>(Visualization::hiveGetPCLVisualizer());
    m_WindowUI.VTKWidget->SetRenderWindow(pViewer->getRenderWindow());
    pViewer->setupInteractor(m_WindowUI.VTKWidget->GetInteractor(), m_WindowUI.VTKWidget->GetRenderWindow());
    m_WindowUI.VTKWidget->update();
}

void CSingleStepWindow::__initialSlider()
{
    auto pSubWindow = new QMdiSubWindow(m_WindowUI.VTKWidget);

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

void CSingleStepWindow::__parseConfigFile()
{
    const std::string ConfigPath = "PointCloudRetouchConfig.xml";
    m_pPointCloudRetouchConfig = new hiveObliquePhotography::PointCloudRetouch::CPointCloudRetouchConfig;
    if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, m_pPointCloudRetouchConfig) != hiveConfig::EParseResult::SUCCEED)
    {
        _HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
        return;
    }
}

std::string CSingleStepWindow::__getFileName(const std::string& vFilePath)
{
    return vFilePath.substr(vFilePath.find_last_of('/') + 1, vFilePath.find_last_of('.') - vFilePath.find_last_of('/') - 1);
}

std::string CSingleStepWindow::__getDirectory(const std::string& vFilePath)
{
    return vFilePath.substr(0, vFilePath.find_last_of('/'));
}

void CSingleStepWindow::__onActionOpen()
{
    QStringList FilePathList = QFileDialog::getOpenFileNames(this, tr("Open PointCloud"), QString::fromStdString(m_CloudPath), tr("PointCloud Files(*.pcd *.ply)"));
    std::vector<std::string> FilePathSet;

    if (FilePathList.empty())
        return;

    foreach(QString FilePathQString, FilePathList)
    {
        std::string FilePathString = FilePathQString.toStdString();
        FilePathSet.push_back(FilePathString);
    }

    m_CloudPath = FilePathSet.front().substr(0, FilePathSet.front().find_last_of("/"));

    __loadCloud(FilePathSet);
}

void CSingleStepWindow::__onActionLoad()
{
    QStringList FilePathList = QFileDialog::getOpenFileNames(this, tr("Open Indices"), QString::fromStdString(m_IndicesPath), tr("Open PointCloud Indices(*.txt)"));
    std::vector<std::string> IndicesPathSet;

    if (FilePathList.empty())
        return;

    foreach(QString FilePathQString, FilePathList)
    {
        std::string FilePathString = FilePathQString.toStdString();
        IndicesPathSet.push_back(FilePathString);
    }

    for (auto& IndicesPath : IndicesPathSet)
    {
        auto RandomColor = hiveMath::hiveGenerateRandomIntegerSet(100, 255, 3);
        Visualization::hiveHighlightPointSet(__loadIndices(IndicesPath), { RandomColor[0], RandomColor[1], RandomColor[2] });
    }

    m_IndicesPath = IndicesPathSet.front().substr(0, IndicesPathSet.front().find_last_of("/"));

    std::vector<std::size_t> PointLabel;
    PointCloudRetouch::hiveDumpPointLabel(PointLabel);
    Visualization::hiveRefreshVisualizer(PointLabel);
}

void CSingleStepWindow::__onActionClear()
{
    Visualization::hiveCancelAllHighlighting();
    std::vector<std::size_t> PointLabel;
    PointCloudRetouch::hiveDumpPointLabel(PointLabel);
    Visualization::hiveRefreshVisualizer(PointLabel);
}

void CSingleStepWindow::__loadCloud(const std::vector<std::string>& vFilePathSet)
{
    m_pCloudSet = hiveInitPointCloudScene(vFilePathSet);

    if (!m_pCloudSet.empty())
    {
        PointCloudRetouch::hiveInit(m_pCloudSet, m_pPointCloudRetouchConfig);
        Visualization::hiveInitVisualizer(m_pCloudSet, true);
        CSingleStepWindow::__initialVTKWidget();
        std::vector<std::size_t> PointLabel;
        PointCloudRetouch::hiveDumpPointLabel(PointLabel);
        Visualization::hiveRefreshVisualizer(PointLabel, true);
        CSingleStepWindow::__initialSlider();
    }
}

std::vector<int> CSingleStepWindow::__loadIndices(const std::string& vPath)
{
    std::vector<int> Indices;
    const std::string Path{ vPath };
    std::ifstream File(Path);
    boost::archive::text_iarchive ia(File);
    ia >> BOOST_SERIALIZATION_NVP(Indices);
    File.close();
    return Indices;
}