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
			CSliderSizeDockWidget(QWidget* vParent)
				: QDockWidget(vParent),
				m_pUi(std::make_shared<Ui::CSliderSizeDockWidget>())
			{
				m_pUi->setupUi(this);
				this->setFixedSize(235, 90);
				QPoint ParentPoint = vParent->pos();
				QPoint p1 = vParent->mapToGlobal(ParentPoint);
				this->move(p1.x() + vParent->width() - this->width() - 12, p1.y() - 12);
			}

		private:
			std::shared_ptr<Ui::CSliderSizeDockWidget> m_pUi = nullptr;

		public slots:

		};
	}
}