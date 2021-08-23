#pragma once
#include "ui_TextureSynthesizerWindow.h"

class QSlider;

class CTextureSynthesizerWindow : public QMainWindow
{
    Q_OBJECT

public:
    CTextureSynthesizerWindow(QWidget* vParent = Q_NULLPTR);
    ~CTextureSynthesizerWindow();

private:
    Ui::CTextureSynthesizerWindow m_WindowUI;

    QSlider* m_pPointSizeSlider = nullptr;
    int m_PointSize = 3;             // magic
    std::vector<std::string> m_FilePathList;
    std::string m_CloudPath = "../TestModel/General";
    std::string m_IndicesPath = "../TestModel/Indices";

    void __connectSignals();
    std::string __getFileName(const std::string& vFilePath);
    std::string __getDirectory(const std::string& vFilePath);

private slots:
    void __onActionLoadInput();
    void __onActionLoadOutput();
    void __onActionLoadMask();
    void __onActionExecute();
};
