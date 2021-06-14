#pragma once
#include "ui_QTInterface.h"
#include "QTDockWidgetTitleBar.h"

class QStandardItem;
class QStandardItemModel;
class QSlider;

namespace hiveObliquePhotography
{
    namespace QTInterface

    {
        class QTInterface : public QMainWindow
        {
            Q_OBJECT

        public:
            QTInterface(QWidget* vParent = Q_NULLPTR);
            ~QTInterface();
            void closeEvent(QCloseEvent* vEvent) override;

        private:
            Ui::QTInterfaceClass ui;

            QStandardItemModel* m_pResourceSpaceStandardItemModels = nullptr;
            QStandardItemModel* m_pWorkSpaceStandardItemModels = nullptr;
            QSlider* m_pPointSizeSlider = nullptr;
            std::string m_CurrentCloud = "";
            std::string m_DirectoryOpenPath = "../Models/Tile1";
            size_t m_SceneIndex = -1;
            int m_PointSize = 3;             // magic
            std::vector<std::string> m_FilePathList;

            void __initialVTKWidget();
            void __initialResourceSpaceDockWidget();
            void __initialWorkSpaceDockWidget();
            void __initialMessageDockWidget();
            void __initialDockWidgetTitleBar(QDockWidget* vParentWidget, const std::string& vTitleBarText);
            void __initialSlider(const QStringList& vFilePathList);
            bool __addResourceSpaceCloudItem(const std::string& vFilePath);
            bool __deleteResourceSpaceCloudItem(const std::string& vFilePath);
            bool __MessageDockWidgetOutputText(QString vString);
            bool __parseConfigFile();
            void __connectSignals();
            void __checkFileOpenRepeatedly();
            template <class T>
            bool __readConfigFile(const std::string& vFileName, T* vInstance);
            std::string __getFileName(const std::string& vFilePath);
            std::string __getDirectory(const std::string& vFilePath);

        private slots:
            void onActionOpen();
            void onActionSetting();
            void onActionResetSelectStatus();
            void onActionTest();
            void onResourceSpaceItemDoubleClick(const QModelIndex& vIndex);

        };
    }
}

