#pragma once
#include "ui_TextureSynthesizerWindow.h"
#include "TreeBasedTextureSynthesizer.h"

using namespace hiveObliquePhotography::PointCloudRetouch;

class CTextureSynthesizerWindow : public QMainWindow
{
    Q_OBJECT

public:
    CTextureSynthesizerWindow(QWidget* vParent = Q_NULLPTR);
    ~CTextureSynthesizerWindow();

private:
    Ui::CTextureSynthesizerWindow m_WindowUI;

    std::vector<std::string> m_FilePathList;
    std::string m_Path = "../TestModel/Test004";

    Eigen::Matrix<Eigen::Vector3i, -1, -1> m_Input;
    Eigen::Matrix<Eigen::Vector3i, -1, -1> m_Output;
    Eigen::MatrixXi m_Mask;
    std::string m_OutputName;

    void __connectSignals();
    void __loadImage(const std::string& vImagePath, Eigen::Matrix<Eigen::Vector3i, -1, -1>& voTexture);
    void __loadMask(const std::string& vImagePath, Eigen::MatrixXi& voMask);
    void __saveImage(const Eigen::Matrix<Eigen::Vector3i, -1, -1>& vTexture, const std::string& vOutputImagePath);

private slots:
    void __onActionLoadInput();
    void __onActionLoadOutput();
    void __onActionLoadMask();
    void __onActionExecute();
};
