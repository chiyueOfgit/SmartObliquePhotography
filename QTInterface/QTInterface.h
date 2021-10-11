#pragma once
#include "ui_QTInterface.h"
#include "QTDockWidgetTitleBar.h"
#include "InstructionsDialog.h"
#include "Mesh.h"

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
        struct STileSet
        {
            std::vector<std::string> NameSet;
            std::vector<PointCloud_t::Ptr> TileSet;
        };

        struct SMeshSet
        {
            std::vector<std::string> NameSet;
            std::vector<CMesh> MeshSet;
        };

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
            std::string m_CloudOpenPath = "../Models";
            std::string m_MeshOpenPath = "../UnitTests/TestData/Test026_Model";
            int m_PointSize = 3;             // magic

            STileSet m_TileSet;
            SMeshSet m_MeshSet;
            std::set<int> m_SelectedTileIndices;
            std::set<int> m_SelectedMeshIndices;

            QDockWidget* m_pRubberSizeDockWidget = nullptr;
            QDockWidget* m_pBrushSizeDockWidget = nullptr;
            QDockWidget* m_pPointPickingDockWidget = nullptr;
            QMdiSubWindow* m_pAreaPickingSetting = nullptr;
            CInstructionsDialog* m_pInstructionsDialog = nullptr;
            CDisplayOptionsSettingDialog* m_pDisplayOptionsSettingDialog = nullptr;
            vtkSmartPointer<vtkOrientationMarkerWidget> m_pAxes;
        	
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
            bool __messageDockWidgetOutputText(const std::string& vText);
            bool __addResourceSpaceCloudItem(const std::string& vFileName);
            bool __addResourceSpaceMeshItem(const std::string& vFileName);
            void __addAxesToView(vtkRenderWindowInteractor* vInteractor, double vX, double vY, double vXWide, double vYWide);
        	
            std::string __getFileName(const std::string& vFilePath);
            std::string __getDirectory(const std::string& vFilePath);
            std::string __getFileNameWithSuffix(const std::string& vFilePath);
            std::string __getSuffix(const std::string& vFilePath);

        private slots:
            void onActionPointPicking();
            void onActionAreaPicking();
            void onActionOpen();
            void onActionOpenMesh();
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
            void onActionSutureMesh();
            void onActionReconstruction();
            void onActionParameterization();
            void onActionBakeTexture();
            void onResourceSpaceItemDoubleClick(QModelIndex);
            void onResourceSpaceItemClick(QModelIndex vIndex);
            void onResourceSpaceItemChange(QStandardItem* vItem);
        };
    }
}

