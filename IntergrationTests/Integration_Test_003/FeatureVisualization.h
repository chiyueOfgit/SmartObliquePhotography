#pragma once
#include "ui_FeatureVisualization.h"

class QStandardItem;
class QStandardItemModel;
class QSlider;

namespace hiveObliquePhotography
{
    namespace Visualization
    {
        class CVisualizationConfig;
    }

    namespace FeatureVisualization
    {
        class CFeatureVisualization : public QMainWindow
        {
            Q_OBJECT

        public:
            CFeatureVisualization(QWidget* vParent = Q_NULLPTR);
            ~CFeatureVisualization();

            void init();
            void closeEvent(QCloseEvent* vEvent) override;

        private:
            Ui::CFeatureVisualizationClass ui;

            QSlider* m_pPointSizeSlider = nullptr;
            std::string m_CurrentCloud = "";
            std::string m_DirectoryOpenPath = "../Models";
            size_t m_SceneIndex = -1;
            int m_PointSize = 3;
            pcl::PointCloud<pcl::PointSurfel>::Ptr m_pCloud = nullptr;
            QDockWidget* m_pPointPickingDockWidget = nullptr;

            Visualization::CVisualizationConfig* m_pVisualizationConfig = nullptr;
            hiveConfig::CHiveConfig* m_pPointCloudRetouchConfig = nullptr;

            void __initialVTKWidget();
            void __initialSlider(const QStringList& vFilePathList);
            void __connectSignals();
            void __parseConfigFile();

            std::string __getFileName(const std::string& vFilePath);
            std::string __getDirectory(const std::string& vFilePath);

        private slots:
            void onActionPointPicking();
            void onActionOpen();
            void onActionDiscardAndRecover();
            void onActionDelete();
            void onActionRubber();
            void onActionOutlierDetection();
        };
    }
}

