#pragma once
#include "qspinbox.h"

class QFloatSpinBox : public QDoubleSpinBox
{
	Q_OBJECT

public:

	QFloatSpinBox(QWidget* pParent = NULL);;

	virtual QSize sizeHint() const;
	void setValue(double Value, bool BlockSignals = false);
};

QSize QFloatSpinBox::sizeHint() const
{
	return QSize(90, 20);
}

QFloatSpinBox::QFloatSpinBox(QWidget* pParent /*= NULL*/) :
	QDoubleSpinBox(pParent)
{
}

void QFloatSpinBox::setValue(double Value, bool BlockSignals)
{
	blockSignals(BlockSignals);

	QDoubleSpinBox::setValue(Value);

	blockSignals(false);
}