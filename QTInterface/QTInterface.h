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
            QTDockWidgetTitleBar* m_pDockWidgetTitleBar = nullptr;
            QSlider* m_pPointSizeSlider = nullptr;
            std::string m_CurrentCloud = "";

            void __initialVTKWidget();
            void __initialResourceDockWidget();
            void __initialWorkSpaceDockWidget();
            void __initialMessageDockWidget();
            void __initialDockWidgetTitleBar();
            void __initialSlider(const QStringList& vFileNameList);
            void __connectSignals();
            std::string __getFileName(const std::string& vPath);


        private slots:
            void onActionOpen();
            void onActionSetting();
            void onResourceSpaceItemDoubleClick(const QModelIndex& vIndex);

        };
    }
}

