#include "pch.h"
#include "SliderSizeDockWidget.h"

using namespace hiveObliquePhotography::QTInterface;

void hiveObliquePhotography::QTInterface::CSliderSizeDockWidget::__setSize()
{
	this->setFixedSize(235, 90);
}

void hiveObliquePhotography::QTInterface::CSliderSizeDockWidget::__setPosition(QWidget* vParent)
{
	QPoint ParentPoint = vParent->pos();
	QPoint p1 = vParent->mapToGlobal(ParentPoint);
	this->move(p1.x() + vParent->width() - this->width() - 12, p1.y() - 12);
}

