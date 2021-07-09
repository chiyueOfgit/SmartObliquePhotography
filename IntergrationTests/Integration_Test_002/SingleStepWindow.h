#pragma once
#include "ui_SingleStepWindow.h"
#include "VisualizationConfig.h"

class QSlider;

class CSingleStepWindow : public QMainWindow
{
    Q_OBJECT

public:
    CSingleStepWindow(QWidget* vParent = Q_NULLPTR);
    ~CSingleStepWindow();

private:
    Ui::CSingleStepWindow m_WindowUI;

    QSlider* m_pPointSizeSlider = nullptr;
    int m_PointSize = 3;             // magic
    std::vector<std::string> m_FilePathList;
    std::string m_CloudPath = "../TestModel/General";
    PointCloud_t::Ptr m_pCloud = nullptr;

    hiveObliquePhotography::Visualization::CVisualizationConfig* m_pVisualizationConfig = nullptr;
    hiveConfig::CHiveConfig* m_pPointCloudRetouchConfig = nullptr;
    std::vector<std::size_t> m_PointLabel;
    std::vector<pcl::index_t> m_ExpandPoints;

    std::size_t m_StepLength = 500;  //每次步长点数
    Eigen::Vector3i m_BeginColor = { 0, 0, 255 };
    Eigen::Vector3i m_EndColor = { 255, 0, 0 };

    void __loadCloud(const std::vector<std::string>& vFilePathSet);
    void __initialVTKWidget();
    void __initialSlider();
    void __connectSignals();
    void __parseConfigFile();

    std::string __getFileName(const std::string& vFilePath);
    std::string __getDirectory(const std::string& vFilePath);

private slots:
    void __onActionOpen();
    void __onActionMark();
    void __onActionShow();
    void __onActionClear();
};
