#pragma once
#include "ui_QTInterface.h"
#include "QTDockWidgetTitleBar.h"

class QStandardItem;
class QStandardItemModel;
class QSlider;

namespace hiveObliquePhotography
{
    namespace Visualization
    {
        class CVisualizationConfig;
    }

    namespace QTInterface
    {
        class QTInterface : public QMainWindow
        {
            Q_OBJECT

        public:
            QTInterface(QWidget* vParent = Q_NULLPTR);
            ~QTInterface();

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
            pcl::PointCloud<pcl::PointSurfel>::Ptr m_pCloud = nullptr;
            QDockWidget* m_pRubberSizeDockWidget = nullptr;
            QDockWidget* m_pBrushSizeDockWidget = nullptr;
            QDockWidget* m_pPointPickingDockWidget = nullptr;

            Visualization::CVisualizationConfig* m_pVisualizationConfig = nullptr;
            hiveConfig::CHiveConfig* m_pPointCloudRetouchConfig = nullptr;
            //PointCloudRetouch::CPointCloudRetouchConfig* m_pPointCloudRetouchConfig = nullptr;

            void __initialVTKWidget();
            void __initialResourceSpaceDockWidget();
            void __initialWorkSpaceDockWidget();
            void __initialMessageDockWidget();
            void __initialDockWidgetTitleBar(QDockWidget* vParentWidget, const std::string& vTitleBarText);
            void __initialSlider(const QStringList& vFilePathList);
            void __connectSignals();
            void __parseConfigFile();
            bool __messageDockWidgetOutputText(QString vString);
            std::string __getFileName(const std::string& vFilePath);
            std::string __getDirectory(const std::string& vFilePath);

        private slots:
            void onActionPointPicking();
            void onActionOpen();
        };
    }
}

