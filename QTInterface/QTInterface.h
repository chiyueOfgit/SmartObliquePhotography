#pragma once
#include "ui_QTInterface.h"
#include "QTDockWidgetTitleBar.h"
#include "InstructionsDialog.h"

class QStandardItem;
class QStandardItemModel;
class QSlider;
class QMdiSubWindow;
class QCheckBox;

namespace hiveObliquePhotography
{
    namespace Visualization
    {
        class CVisualizationConfig;
    }

    namespace QTInterface
    {
        class CDisplayOptionsSettingDialog;

        class CQTInterface : public QMainWindow
        {
            Q_OBJECT

        public:
            CQTInterface(QWidget* vParent = Q_NULLPTR);
            ~CQTInterface();

            void init();
            void closeEvent(QCloseEvent* vEvent) override;

        protected:
            virtual void keyPressEvent(QKeyEvent* vEvent) override;

        private:
            Ui::CQTInterfaceClass m_UI;

            QStandardItemModel* m_pResourceSpaceStandardItemModels = nullptr;
            QStandardItemModel* m_pWorkSpaceStandardItemModels = nullptr;
            QSlider* m_pPointSizeSlider = nullptr;
            QCheckBox* m_pAreaPickingCullingBox = nullptr;
            std::string m_CurrentCloud = "";
            std::string m_DirectoryOpenPath = "../Models";
            size_t m_SceneIndex = -1;
            int m_PointSize = 3;             // magic
            std::vector<std::string> m_FilePathList;
            std::vector<PointCloud_t::Ptr> m_TileSet;
            QDockWidget* m_pRubberSizeDockWidget = nullptr;
            QDockWidget* m_pBrushSizeDockWidget = nullptr;
            QDockWidget* m_pPointPickingDockWidget = nullptr;
            QMdiSubWindow* m_pAreaPickingSetting = nullptr;
            CInstructionsDialog* m_pInstructionsDialog = nullptr;
            CDisplayOptionsSettingDialog* m_pDisplayOptionsSettingDialog = nullptr;
            vtkSmartPointer<vtkOrientationMarkerWidget> m_Axes;
        	
            Visualization::CVisualizationConfig* m_pVisualizationConfig = nullptr;
            hiveConfig::CHiveConfig* m_pPointCloudRetouchConfig = nullptr;

            void __initialVTKWidget();
            void __initialResourceSpaceDockWidget();
            void __initialWorkSpaceDockWidget();
            void __initialMessageDockWidget();
            void __initialDockWidgetTitleBar(QDockWidget* vParentWidget, const std::string& vTitleBarText);
            void __initialSlider(const QStringList& vFilePathList);
            void __connectSignals();
            void __parseConfigFile();
            bool __messageDockWidgetOutputText(QString vString);
            bool __addResourceSpaceCloudItem(const std::string& vFilePath);
            void __addAxesToView(vtkRenderWindowInteractor* vInteractor, double vX, double vY, double vXWide, double vYWide);
        	
            std::string __getFileName(const std::string& vFilePath);
            std::string __getDirectory(const std::string& vFilePath);

        private slots:
            void onActionPointPicking();
            void onActionAreaPicking();
            void onActionOpen();
            void onActionSave();
            void onActionDiscardAndRecover();
            void onActionDeleteLitter();
            void onActionDeleteBackground();
            void onActionPrecompute();
            void onActionRubber();
            void onActionBrush();
            void onActionOutlierDetection();
            void onActionStartRepairHole();
            void onActionInstructions();
            void onActionSetting();
            void onActionOpenMesh();
            void onResourceSpaceItemDoubleClick(QModelIndex);
        };
    }
}

