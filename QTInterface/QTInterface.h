#pragma once
#include "ui_QTInterface.h"
#include "PointCloudVisualizer.h"
#include "PointCloudScene.h"

namespace hiveObliquePhotography
{
    namespace QTInterface
    {
        class CInteractionManager;

        class QTInterface : public QMainWindow
        {
            Q_OBJECT

        public:
            QTInterface(QWidget* vParent = Q_NULLPTR);
            ~QTInterface();

        private:
            Ui::QTInterfaceClass ui;

            void __initialVTKWidget();
            void __connectSignals();


        private slots:
            void onActionOpen();

        };
    }
}

