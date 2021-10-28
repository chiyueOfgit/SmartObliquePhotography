#pragma once
#include "ui_MeshHighlight.h"
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

    namespace MeshHighlight
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

        class CMeshHighlight : public QMainWindow
        {
            Q_OBJECT

        public:
            CMeshHighlight(QWidget* vParent = Q_NULLPTR);
            ~CMeshHighlight();

            void init();
            void closeEvent(QCloseEvent* vEvent) override;

        protected:
            virtual void keyPressEvent(QKeyEvent* vEvent) override;

        private:
            Ui::CMeshHighlightClass m_UI;

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

            vtkSmartPointer<vtkOrientationMarkerWidget> m_pAxes;
        	
            Visualization::CVisualizationConfig* m_pVisualizationConfig = nullptr;
            hiveConfig::CHiveConfig* m_pPointCloudRetouchConfig = nullptr;

            void __initialVTKWidget();
            void __connectSignals();
            void __parseConfigFile();
            void __addAxesToView(vtkRenderWindowInteractor* vInteractor, double vX, double vY, double vXWide, double vYWide);
        	
            std::string __getFileName(const std::string& vFilePath);
            std::string __getDirectory(const std::string& vFilePath);
            std::string __getFileNameWithSuffix(const std::string& vFilePath);
            std::string __getSuffix(const std::string& vFilePath);

        private slots:
            void onActionOpenMesh();
            void onActionSave();
            void onActionSutureMesh();
            void onActionShowMeshIndices();
            void onActionReconstruction();
            void onActionParameterization();
            void onActionBakeTexture();
        };
    }
}

