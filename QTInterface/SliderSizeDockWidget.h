#pragma once
#include "ui_SliderSizeDockWidget.h"
#include "QTInterface.h"
#include <QObject>
#include <qdockwidget.h>
#include <qwidget.h>
#include <qstyle.h>

namespace hiveObliquePhotography
{
	namespace QTInterface
	{
		class CSliderSizeDockWidget : public QDockWidget
		{
			Q_OBJECT

		public:
			CSliderSizeDockWidget(QWidget* vParent, hiveConfig::CHiveConfig* vPointCloudRetouchConfig)
				: QDockWidget(vParent),
				m_pPointCloudRetouchConfig(vPointCloudRetouchConfig),
				m_pUi(std::make_shared<Ui::CSliderSizeDockWidget>())
			{
				m_pUi->setupUi(this);
				__setSize();
				__setPosition(vParent);

				//m_pUi->SliderRadius->setValue(m_pPointCloudRetouchConfig->getAttribute<float>("RADIUS").value());

			}

		private:
			std::shared_ptr<Ui::CSliderSizeDockWidget> m_pUi = nullptr;
			hiveConfig::CHiveConfig* m_pPointCloudRetouchConfig = nullptr;

			void __setSize();
			void __setPosition(QWidget* vParent);

		public slots:

		};
	}
}