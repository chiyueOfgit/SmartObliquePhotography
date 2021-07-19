#include "pch.h"
#include "FeatureVisualization.h"
#include <QtWidgets/qmdisubwindow.h>
#include <QtWidgets/qslider.h>
#include <QtWidgets/QFileDialog>
#include <QStandardItem>
#include <qobject.h>
#include <vtkRenderWindow.h>
#include <QDateTime>
#include <QColor>
#include <QtWidgets/qlabel.h>
#include <iostream>
#include <string>
#include <QtWidgets/qmainwindow.h>
#include <vtkAutoInit.h>
#include <algorithm>
#include <tuple>
#include <typeinfo>
#include <QtWidgets/qpushbutton.h>
#include <qcursor.h>
#include <qevent.h>
#include <qpainter.h>

#include "SliderSizeDockWidget.h"
#include "ObliquePhotographyDataInterface.h"
#include "PointCloudRetouchInterface.h"
#include "PointCloudRetouchConfig.h"
#include "PointCloudVisualizer.h"
#include "InteractionCallback.h"
#include "VisualizationConfig.h"
#include "NormalComplexityVisualization.h"
#include "ColorVisualization.h"

#include "pcl/io/pcd_io.h"

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

using namespace hiveObliquePhotography::FeatureVisualization;

CFeatureVisualization::CFeatureVisualization(QWidget * vParent)
    : QMainWindow(vParent)
{
    ui.setupUi(this);
}

CFeatureVisualization::~CFeatureVisualization()
{
}

void CFeatureVisualization::init()
{
    {
        m_pVisualizationConfig = hiveObliquePhotography::Visualization::CVisualizationConfig::getInstance();
    }

    __connectSignals();
    __parseConfigFile();
}

void CFeatureVisualization::__connectSignals()
{
    QObject::connect(ui.actionOpen, SIGNAL(triggered()), this, SLOT(onActionOpen()));
    QObject::connect(ui.actionPointPicking, SIGNAL(triggered()), this, SLOT(onActionPointPicking()));
    QObject::connect(ui.actionUpdate, SIGNAL(triggered()), this, SLOT(onActionDiscardAndRecover()));
    QObject::connect(ui.actionDelete, SIGNAL(triggered()), this, SLOT(onActionDelete()));
    QObject::connect(ui.actionRubber, SIGNAL(triggered()), this, SLOT(onActionRubber()));
    QObject::connect(ui.actionOutlierDetection, SIGNAL(triggered()), this, SLOT(onActionOutlierDetection()));
    QObject::connect(ui.actionColor, SIGNAL(triggered()), this, SLOT(onActionFeatureColor()));
    QObject::connect(ui.actionNormalComplexity, SIGNAL(triggered()), this, SLOT(onActionFeatureNormalComplexity()));
    QObject::connect(ui.actionPlanarity, SIGNAL(triggered()), this, SLOT(onActionFeaturePlanarity()));
}

void CFeatureVisualization::__initialVTKWidget()
{
    auto pViewer = static_cast<pcl::visualization::PCLVisualizer*>(hiveObliquePhotography::Visualization::CPointCloudVisualizer::getInstance()->getVisualizer());
    ui.VTKWidget->SetRenderWindow(pViewer->getRenderWindow());
    pViewer->setupInteractor(ui.VTKWidget->GetInteractor(), ui.VTKWidget->GetRenderWindow());
    ui.VTKWidget->update();
}

void CFeatureVisualization::__initialSlider(const QStringList& vFilePathList)
{
    const std::string& FirstCloudFilePath = vFilePathList[0].toStdString();
    auto FileCloudFileName = CFeatureVisualization::__getFileName(FirstCloudFilePath);

    auto pSubWindow = new QMdiSubWindow(ui.VTKWidget);

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
                Visualization::CPointCloudVisualizer::getInstance()->refresh(PointLabel);
            }
        }
    );

    pSubWindow->setWidget(m_pPointSizeSlider);
    pSubWindow->resize(200, 50);                // magic
    pSubWindow->setWindowFlag(Qt::FramelessWindowHint);
    pSubWindow->show();
}

void CFeatureVisualization::__parseConfigFile()
{
    const std::string ConfigPath = "PointCloudRetouchConfig.xml";
    m_pPointCloudRetouchConfig = new hiveObliquePhotography::PointCloudRetouch::CPointCloudRetouchConfig;
    if (hiveConfig::hiveParseConfig(ConfigPath, hiveConfig::EConfigType::XML, m_pPointCloudRetouchConfig) != hiveConfig::EParseResult::SUCCEED)
    {
        _HIVE_OUTPUT_WARNING(_FORMAT_STR1("Failed to parse config file [%1%].", ConfigPath));
        return;
    }
}

std::string CFeatureVisualization::__getFileName(const std::string& vFilePath)
{
    return vFilePath.substr(vFilePath.find_last_of('/') + 1, vFilePath.find_last_of('.') - vFilePath.find_last_of('/') - 1);
}

std::string CFeatureVisualization::__getDirectory(const std::string& vFilePath)
{
    return vFilePath.substr(0, vFilePath.find_last_of('/'));
}

void CFeatureVisualization::onActionPointPicking()
{
    if (m_pCloud)
    {
        if (ui.actionPointPicking->isChecked())
        {
            m_pPointPickingDockWidget = new CSliderSizeDockWidget(ui.VTKWidget, m_pVisualizationConfig);
            m_pPointPickingDockWidget->setWindowTitle(QString("Point Picking"));
            m_pPointPickingDockWidget->show();
        }
        else
        {
            m_pPointPickingDockWidget->close();
            delete m_pPointPickingDockWidget;

            std::vector<std::size_t> PointLabel;
            PointCloudRetouch::hiveDumpPointLabel(PointLabel);
            Visualization::CPointCloudVisualizer::getInstance()->refresh(PointLabel);
        }

        if (m_pVisualizationConfig)
            m_pVisualizationConfig->overwriteAttribute(Visualization::CIRCLE_MODE, ui.actionPointPicking->isChecked());
    }

}

void CFeatureVisualization::onActionOpen()
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
        m_DirectoryOpenPath = CFeatureVisualization::__getDirectory(FilePathSet.back());

        auto config = m_pPointCloudRetouchConfig->getSubconfigAt(0);
        auto num = config->getNumSubconfig();

        PointCloudRetouch::hiveInit(m_pCloud, m_pPointCloudRetouchConfig);
        Visualization::CPointCloudVisualizer::getInstance()->init(m_pCloud, true);
        //Visualization::hiveRegisterQTLinker(new CQTLinker(this));
        CFeatureVisualization::__initialVTKWidget();
        std::vector<std::size_t> PointLabel;
        PointCloudRetouch::hiveDumpPointLabel(PointLabel);
        Visualization::CPointCloudVisualizer::getInstance()->refresh(PointLabel, true);
        CFeatureVisualization::__initialSlider(FilePathList);
    }
}

void CFeatureVisualization::onActionRubber()
{
    if (m_pVisualizationConfig)
    {
        m_pVisualizationConfig->overwriteAttribute("RUBBER_MODE", ui.actionRubber->isChecked());
    }
}

void CFeatureVisualization::onActionOutlierDetection()
{
    if (m_pCloud)
    {
        PointCloudRetouch::hiveMarkIsolatedAreaAsLitter();
        std::vector<std::size_t> PointLabel;
        PointCloudRetouch::hiveDumpPointLabel(PointLabel);
        Visualization::CPointCloudVisualizer::getInstance()->refresh(PointLabel);
    }
}

void CFeatureVisualization::onActionDiscardAndRecover()
{
    static int i = 1;
    if (i++ % 2)
        PointCloudRetouch::hiveHideLitter();
    else
        PointCloudRetouch::hiveDisplayLitter();

    std::vector<std::size_t> PointLabel;
    PointCloudRetouch::hiveDumpPointLabel(PointLabel);
    Visualization::CPointCloudVisualizer::getInstance()->refresh(PointLabel);
}

void CFeatureVisualization::onActionDelete()
{
    PointCloudRetouch::hiveClearMark();
    Visualization::CPointCloudVisualizer::getInstance()->removeAllUserColoredPoints();
    std::vector<std::size_t> PointLabel;
    PointCloudRetouch::hiveDumpPointLabel(PointLabel);
    Visualization::CPointCloudVisualizer::getInstance()->refresh(PointLabel);
}

void CFeatureVisualization::onActionFeatureColor()
{
    Visualization::CPointCloudVisualizer::getInstance()->setFeatureMode(Visualization::EFeatureMode::ColorFeature);
}

void CFeatureVisualization::onActionFeatureNormalComplexity()
{
    Feature::CNormalComplexityVisualization::getInstance()->init(m_pCloud);
    Feature::CNormalComplexityVisualization::getInstance()->run();
    std::vector<std::size_t> PointLabel;
    PointCloudRetouch::hiveDumpPointLabel(PointLabel);
    Visualization::CPointCloudVisualizer::getInstance()->refresh(PointLabel);
}

void CFeatureVisualization::onActionFeaturePlanarity()
{
    Visualization::CPointCloudVisualizer::getInstance()->setFeatureMode(Visualization::EFeatureMode::PlaneFeature);
}

void CFeatureVisualization::closeEvent(QCloseEvent* vEvent)
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