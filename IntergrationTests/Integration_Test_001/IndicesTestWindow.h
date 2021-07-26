#pragma once
#include "ui_IndicesTestWindow.h"
#include "VisualizationConfig.h"

class QSlider;

class CSingleStepWindow : public QMainWindow
{
    Q_OBJECT

public:
    CSingleStepWindow(QWidget* vParent = Q_NULLPTR);
    ~CSingleStepWindow();

private:
    Ui::CIndicesTestWindow m_WindowUI;

    QSlider* m_pPointSizeSlider = nullptr;
    int m_PointSize = 3;             // magic
    std::vector<std::string> m_FilePathList;
    std::string m_CloudPath = "../TestModel/General";
    std::string m_IndicesPath = "../TestModel/Indices";
    PointCloud_t::Ptr m_pCloud = nullptr;

    hiveObliquePhotography::Visualization::CVisualizationConfig* m_pVisualizationConfig = nullptr;
    hiveConfig::CHiveConfig* m_pPointCloudRetouchConfig = nullptr;

    void __loadCloud(const std::vector<std::string>& vFilePathSet);
    void __initialVTKWidget();
    void __initialSlider();
    void __connectSignals();
    void __parseConfigFile();

    std::vector<int> __loadIndices(const std::string& vPath);
    std::string __getFileName(const std::string& vFilePath);
    std::string __getDirectory(const std::string& vFilePath);

private slots:
    void __onActionOpen();
    void __onActionLoad();
    void __onActionClear();
};
