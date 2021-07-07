#include "pch.h"
#include "TestWindow.h"
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

CTestWindow::CTestWindow(QWidget * vParent)
    : QMainWindow(vParent)
{
    Visualization::hiveGetVisualizationConfig(m_pVisualizationConfig);

    ui.setupUi(this);

    __connectSignals();
    __parseConfigFile();
}

CTestWindow::~CTestWindow()
{
}

void CTestWindow::__connectSignals()
{
    QObject::connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(__onActionOpen()));
    QObject::connect(ui.actionLoad, SIGNAL(triggered()), this, SLOT(__onActionLoad()));
    QObject::connect(ui.actionClear, SIGNAL(triggered()), this, SLOT(__onActionClear()));
}

void CTestWindow::__initialVTKWidget()
{
    auto pViewer = static_cast<pcl::visualization::PCLVisualizer*>(Visualization::hiveGetPCLVisualizer());
    ui.VTKWidget->SetRenderWindow(pViewer->getRenderWindow());
    pViewer->setupInteractor(ui.VTKWidget->GetInteractor(), ui.VTKWidget->GetRenderWindow());
    ui.VTKWidget->update();
}

void CTestWindow::__initialSlider()
{
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

void CTestWindow::__parseConfigFile()
{
    const std::string ConfigPath = "PointCloudRetouchConfig.xml";
    m_pPointCloudRetouchConfig = new hiveObliquePhotography::PointCloudRetouch::CPointCloudRetouchConfig;
    if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, m_pPointCloudRetouchConfig) != hiveConfig::EParseResult::SUCCEED)
    {
        _HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
        return;
    }
}

std::vector<int> CTestWindow::__loadIndices(const std::string& vPath)
{
    std::vector<int> Indices;
    const std::string Path{ vPath };
    std::ifstream File(Path);
    boost::archive::text_iarchive ia(File);
    ia >> BOOST_SERIALIZATION_NVP(Indices);
    File.close();
    return Indices;
}

std::string CTestWindow::__getFileName(const std::string& vFilePath)
{
    return vFilePath.substr(vFilePath.find_last_of('/') + 1, vFilePath.find_last_of('.') - vFilePath.find_last_of('/') - 1);
}

std::string CTestWindow::__getDirectory(const std::string& vFilePath)
{
    return vFilePath.substr(0, vFilePath.find_last_of('/'));
}

void CTestWindow::__onActionOpen()
{
    QStringList FilePathList = QFileDialog::getOpenFileNames(this, tr("Open PointCloud"), QString::fromStdString(m_CloudPath), tr("Open PointCloud files(*.pcd)"));
    std::vector<std::string> FilePathSet;

    if (FilePathList.empty())
        return;

    foreach(QString FilePathQString, FilePathList)
    {
        std::string FilePathString = FilePathQString.toStdString();
        FilePathSet.push_back(FilePathString);
    }

    if (FilePathSet.empty())
        return;

    __loadCloud(FilePathSet);
}

void CTestWindow::__onActionLoad()
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

    if (IndicesPathSet.empty())
        return;

    for (auto& IndicesPath : IndicesPathSet)
    {
        auto RandomColor = hiveMath::hiveGenerateRandomIntegerSet(100, 255, 3);
        Visualization::hiveSetPointsColor(__loadIndices(IndicesPath), { RandomColor[0], RandomColor[1], RandomColor[2] });
    }

    std::vector<std::size_t> PointLabel;
    PointCloudRetouch::hiveDumpPointLabel(PointLabel);
    Visualization::hiveRefreshVisualizer(PointLabel);
}

void CTestWindow::__onActionClear()
{
    Visualization::hiveClearPointsColor();
    std::vector<std::size_t> PointLabel;
    PointCloudRetouch::hiveDumpPointLabel(PointLabel);
    Visualization::hiveRefreshVisualizer(PointLabel);
}

void CTestWindow::__loadCloud(const std::vector<std::string>& vFilePathSet)
{
    m_pCloud = hiveObliquePhotography::hiveInitPointCloudScene(vFilePathSet);

    if (m_pCloud)
    {
        PointCloudRetouch::hiveInit(m_pCloud, m_pPointCloudRetouchConfig);
        Visualization::hiveInitVisualizer(m_pCloud, true);
        CTestWindow::__initialVTKWidget();
        std::vector<std::size_t> PointLabel;
        PointCloudRetouch::hiveDumpPointLabel(PointLabel);
        Visualization::hiveRefreshVisualizer(PointLabel, true);
        CTestWindow::__initialSlider();
    }
}